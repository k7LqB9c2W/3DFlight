#include "terrain/etopo_tile.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

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

bool EtopoTile::LoadFromFile(const std::filesystem::path& path, std::string& error) {
    m_width = 0;
    m_height = 0;
    m_lonWest = 0.0;
    m_lonEast = 0.0;
    m_latSouth = 0.0;
    m_latNorth = 0.0;
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

double EtopoTile::SampleHeightMeters(double latDeg, double lonDeg) const {
    if (!IsValid()) {
        return 0.0;
    }

    NormalizeLon(lonDeg);

    if (latDeg < m_latSouth || latDeg > m_latNorth || lonDeg < m_lonWest || lonDeg > m_lonEast) {
        return 0.0;
    }

    const double u = (lonDeg - m_lonWest) / (m_lonEast - m_lonWest);
    const double v = (m_latNorth - latDeg) / (m_latNorth - m_latSouth);

    const double x = std::clamp(u, 0.0, 1.0) * static_cast<double>(m_width - 1);
    const double y = std::clamp(v, 0.0, 1.0) * static_cast<double>(m_height - 1);

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
    return hx0 + (hx1 - hx0) * ty;
}

} // namespace flight::terrain
