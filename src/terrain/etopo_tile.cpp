#include "terrain/etopo_tile.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <gdal_priv.h>

namespace flight::terrain {
namespace {

struct GeoPoint {
    double lon = 0.0;
    double lat = 0.0;
};

GeoPoint PixelToGeo(const std::array<double, 6>& gt, double x, double y) {
    GeoPoint out{};
    out.lon = gt[0] + x * gt[1] + y * gt[2];
    out.lat = gt[3] + x * gt[4] + y * gt[5];
    return out;
}

void NormalizeLon(double& lonDeg) {
    while (lonDeg >= 180.0) {
        lonDeg -= 360.0;
    }
    while (lonDeg < -180.0) {
        lonDeg += 360.0;
    }
}

} // namespace

const char* PixelMappingModeToString(PixelMappingMode mode) {
    switch (mode) {
        case PixelMappingMode::GeoCanonical:
            return "GeoTransform (lon,lat)";
        case PixelMappingMode::GeoSwapped:
            return "GeoTransform (lat,lon) fallback";
        case PixelMappingMode::KeyBoundsFallback:
            return "Tile-bounds fallback";
        case PixelMappingMode::None:
        default:
            return "None";
    }
}

bool EtopoTile::LoadFromFile(const std::filesystem::path& path, std::string& error) {
    m_width = 0;
    m_height = 0;
    m_lonWest = 0.0;
    m_lonEast = 0.0;
    m_latSouth = 0.0;
    m_latNorth = 0.0;
    m_hasGeoTransform = false;
    m_geoTransform = {};
    m_invGeoTransform = {};
    m_heights.clear();

    if (!std::filesystem::exists(path)) {
        error = "ETOPO tile does not exist: " + path.string();
        return false;
    }

    static bool gdalRegistered = false;
    if (!gdalRegistered) {
        GDALAllRegister();
        gdalRegistered = true;
    }

    GDALDataset* dataset = static_cast<GDALDataset*>(GDALOpen(path.string().c_str(), GA_ReadOnly));
    if (dataset == nullptr) {
        error = "GDALOpen failed for " + path.string();
        return false;
    }

    m_width = dataset->GetRasterXSize();
    m_height = dataset->GetRasterYSize();
    if (m_width <= 1 || m_height <= 1 || dataset->GetRasterCount() < 1) {
        GDALClose(dataset);
        error = "Invalid raster dimensions or missing bands in " + path.string();
        return false;
    }

    GDALRasterBand* band = dataset->GetRasterBand(1);
    if (band == nullptr) {
        GDALClose(dataset);
        error = "Missing band 1 in " + path.string();
        return false;
    }

    m_heights.resize(static_cast<size_t>(m_width) * static_cast<size_t>(m_height));
    CPLErr ioErr = band->RasterIO(
        GF_Read,
        0,
        0,
        m_width,
        m_height,
        m_heights.data(),
        m_width,
        m_height,
        GDT_Float32,
        0,
        0,
        nullptr);
    if (ioErr != CE_None) {
        GDALClose(dataset);
        error = "RasterIO failed for " + path.string();
        return false;
    }

    std::array<double, 6> gt{};
    if (dataset->GetGeoTransform(gt.data()) == CE_None) {
        m_geoTransform = gt;
        m_hasGeoTransform = GDALInvGeoTransform(m_geoTransform.data(), m_invGeoTransform.data()) != 0;
        if (!m_hasGeoTransform) {
            GDALClose(dataset);
            error = "Failed to invert GeoTransform for " + path.string();
            return false;
        }

        const GeoPoint c0 = PixelToGeo(gt, 0.0, 0.0);
        const GeoPoint c1 = PixelToGeo(gt, static_cast<double>(m_width), 0.0);
        const GeoPoint c2 = PixelToGeo(gt, 0.0, static_cast<double>(m_height));
        const GeoPoint c3 = PixelToGeo(gt, static_cast<double>(m_width), static_cast<double>(m_height));

        const double lonMin = std::min({c0.lon, c1.lon, c2.lon, c3.lon});
        const double lonMax = std::max({c0.lon, c1.lon, c2.lon, c3.lon});
        const double latMin = std::min({c0.lat, c1.lat, c2.lat, c3.lat});
        const double latMax = std::max({c0.lat, c1.lat, c2.lat, c3.lat});

        m_lonWest = lonMin;
        m_lonEast = lonMax;
        m_latSouth = latMin;
        m_latNorth = latMax;
    } else {
        GDALClose(dataset);
        error = "GeoTransform not available for " + path.string();
        return false;
    }

    GDALClose(dataset);

    if (m_lonEast <= m_lonWest || m_latNorth <= m_latSouth) {
        error = "Invalid geographic bounds in " + path.string();
        return false;
    }

    return true;
}

void EtopoTile::OverrideBoundsFromTileKey(int latSouthDeg, int lonWestDeg, int tileSpanDeg) {
    m_latSouth = static_cast<double>(latSouthDeg);
    m_latNorth = static_cast<double>(latSouthDeg + tileSpanDeg);
    m_lonWest = static_cast<double>(lonWestDeg);
    m_lonEast = static_cast<double>(lonWestDeg + tileSpanDeg);
}

double EtopoTile::SampleHeightMeters(double latDeg, double lonDeg) const {
    return SampleHeightMetersInternal(latDeg, lonDeg, nullptr);
}

double EtopoTile::SampleHeightMetersDebug(double latDeg, double lonDeg, TileSampleDebug& debug) const {
    return SampleHeightMetersInternal(latDeg, lonDeg, &debug);
}

double EtopoTile::SampleHeightMetersInternal(double latDeg, double lonDeg, TileSampleDebug* debug) const {
    if (debug != nullptr) {
        *debug = {};
        debug->requestLatDeg = latDeg;
        debug->requestLonDeg = lonDeg;
        debug->lonWest = m_lonWest;
        debug->lonEast = m_lonEast;
        debug->latSouth = m_latSouth;
        debug->latNorth = m_latNorth;
        debug->width = m_width;
        debug->height = m_height;
    }

    if (!IsValid()) {
        return 0.0;
    }

    double x = 0.0;
    double y = 0.0;
    PixelMappingMode mappingMode = PixelMappingMode::None;
    double mappedLonDeg = lonDeg;
    double lonWrapAppliedDeg = 0.0;
    if (!MapLatLonToPixel(latDeg, lonDeg, x, y, &mappingMode, &mappedLonDeg, &lonWrapAppliedDeg)) {
        return 0.0;
    }

    const double rawX = x;
    const double rawY = y;
    x = std::clamp(x, 0.0, static_cast<double>(m_width - 1));
    y = std::clamp(y, 0.0, static_cast<double>(m_height - 1));

    const int x0 = std::clamp(static_cast<int>(std::floor(x)), 0, m_width - 1);
    const int y0 = std::clamp(static_cast<int>(std::floor(y)), 0, m_height - 1);
    const int x1 = std::min(x0 + 1, m_width - 1);
    const int y1 = std::min(y0 + 1, m_height - 1);

    const double tx = x - static_cast<double>(x0);
    const double ty = y - static_cast<double>(y0);

    const auto at = [&](int ix, int iy) -> double {
        const size_t idx = static_cast<size_t>(iy) * static_cast<size_t>(m_width) + static_cast<size_t>(ix);
        return static_cast<double>(m_heights[idx]);
    };

    const double h00 = at(x0, y0);
    const double h10 = at(x1, y0);
    const double h01 = at(x0, y1);
    const double h11 = at(x1, y1);

    const double hx0 = h00 + (h10 - h00) * tx;
    const double hx1 = h01 + (h11 - h01) * tx;
    const double sampleHeight = hx0 + (hx1 - hx0) * ty;

    if (debug != nullptr) {
        debug->mapSucceeded = true;
        debug->sampleValid = true;
        debug->mappingMode = mappingMode;
        debug->mappedLonDeg = mappedLonDeg;
        debug->lonWrapAppliedDeg = lonWrapAppliedDeg;
        debug->rawPixelX = rawX;
        debug->rawPixelY = rawY;
        debug->pixelX = x;
        debug->pixelY = y;
        debug->x0 = x0;
        debug->y0 = y0;
        debug->x1 = x1;
        debug->y1 = y1;
        debug->tx = tx;
        debug->ty = ty;
        debug->h00 = h00;
        debug->h10 = h10;
        debug->h01 = h01;
        debug->h11 = h11;
        debug->sampleHeightMeters = sampleHeight;
    }

    return sampleHeight;
}

bool EtopoTile::MapLatLonToPixel(
    double latDeg,
    double lonDeg,
    double& outX,
    double& outY,
    PixelMappingMode* outMode,
    double* outMappedLonDeg,
    double* outLonWrapAppliedDeg) const {
    double normalizedLonDeg = lonDeg;
    NormalizeLon(normalizedLonDeg);

    auto inPixelBounds = [&](double px, double py) -> bool {
        return px >= -0.5 && px <= static_cast<double>(m_width) - 0.5 &&
               py >= -0.5 && py <= static_cast<double>(m_height) - 0.5;
    };

    const std::array<double, 5> lonCandidates = {
        normalizedLonDeg,
        normalizedLonDeg + 360.0,
        normalizedLonDeg - 360.0,
        normalizedLonDeg + 720.0,
        normalizedLonDeg - 720.0,
    };
    double bestDist = std::numeric_limits<double>::max();
    double bestMappedLon = normalizedLonDeg;
    double bestLonWrap = 0.0;
    PixelMappingMode bestMode = PixelMappingMode::None;
    bool found = false;

    if (m_hasGeoTransform) {
        for (double lonCandidate : lonCandidates) {
            // Preferred axis order: x=lon, y=lat.
            {
                const double px = m_invGeoTransform[0] + m_invGeoTransform[1] * lonCandidate + m_invGeoTransform[2] * latDeg;
                const double py = m_invGeoTransform[3] + m_invGeoTransform[4] * lonCandidate + m_invGeoTransform[5] * latDeg;
                if (inPixelBounds(px, py)) {
                    const double dx = px - (static_cast<double>(m_width) * 0.5);
                    const double dy = py - (static_cast<double>(m_height) * 0.5);
                    const double dist = dx * dx + dy * dy;
                    if (!found || dist < bestDist) {
                        found = true;
                        bestDist = dist;
                        outX = px;
                        outY = py;
                        bestMappedLon = lonCandidate;
                        bestLonWrap = lonCandidate - normalizedLonDeg;
                        bestMode = PixelMappingMode::GeoCanonical;
                    }
                }
            }

            // Fallback axis order for files with swapped geospatial axes.
            {
                const double px = m_invGeoTransform[0] + m_invGeoTransform[1] * latDeg + m_invGeoTransform[2] * lonCandidate;
                const double py = m_invGeoTransform[3] + m_invGeoTransform[4] * latDeg + m_invGeoTransform[5] * lonCandidate;
                if (inPixelBounds(px, py)) {
                    const double dx = px - (static_cast<double>(m_width) * 0.5);
                    const double dy = py - (static_cast<double>(m_height) * 0.5);
                    const double dist = dx * dx + dy * dy + 1e9; // prefer canonical axis order when both are valid
                    if (!found || dist < bestDist) {
                        found = true;
                        bestDist = dist;
                        outX = px;
                        outY = py;
                        bestMappedLon = lonCandidate;
                        bestLonWrap = lonCandidate - normalizedLonDeg;
                        bestMode = PixelMappingMode::GeoSwapped;
                    }
                }
            }
        }
    }

    if (found) {
        if (outMode != nullptr) {
            *outMode = bestMode;
        }
        if (outMappedLonDeg != nullptr) {
            *outMappedLonDeg = bestMappedLon;
        }
        if (outLonWrapAppliedDeg != nullptr) {
            *outLonWrapAppliedDeg = bestLonWrap;
        }
        return true;
    }

    // Final fallback: derive pixel from tile key bounds.
    double fallbackLon = std::numeric_limits<double>::quiet_NaN();
    for (double lonCandidate : lonCandidates) {
        if (lonCandidate >= m_lonWest && lonCandidate <= m_lonEast) {
            fallbackLon = lonCandidate;
            break;
        }
    }
    if (latDeg < m_latSouth || latDeg > m_latNorth || !std::isfinite(fallbackLon)) {
        return false;
    }

    const double u = (fallbackLon - m_lonWest) / std::max(1e-9, (m_lonEast - m_lonWest));
    const double v = (m_latNorth - latDeg) / std::max(1e-9, (m_latNorth - m_latSouth));
    outX = std::clamp(u, 0.0, 1.0) * static_cast<double>(m_width - 1);
    outY = std::clamp(v, 0.0, 1.0) * static_cast<double>(m_height - 1);
    if (outMode != nullptr) {
        *outMode = PixelMappingMode::KeyBoundsFallback;
    }
    if (outMappedLonDeg != nullptr) {
        *outMappedLonDeg = fallbackLon;
    }
    if (outLonWrapAppliedDeg != nullptr) {
        *outLonWrapAppliedDeg = fallbackLon - normalizedLonDeg;
    }
    return true;
}

} // namespace flight::terrain
