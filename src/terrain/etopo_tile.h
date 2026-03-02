#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
#include <array>

namespace flight::terrain {

enum class PixelMappingMode {
    None = 0,
    GeoCanonical,
    GeoSwapped,
    KeyBoundsFallback,
};

const char* PixelMappingModeToString(PixelMappingMode mode);

struct TileSampleDebug {
    bool mapSucceeded = false;
    bool sampleValid = false;
    PixelMappingMode mappingMode = PixelMappingMode::None;

    double requestLatDeg = 0.0;
    double requestLonDeg = 0.0;
    double mappedLonDeg = 0.0;
    double lonWrapAppliedDeg = 0.0;

    double rawPixelX = 0.0;
    double rawPixelY = 0.0;
    double pixelX = 0.0;
    double pixelY = 0.0;

    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
    double tx = 0.0;
    double ty = 0.0;

    double h00 = 0.0;
    double h10 = 0.0;
    double h01 = 0.0;
    double h11 = 0.0;
    double sampleHeightMeters = 0.0;

    double lonWest = 0.0;
    double lonEast = 0.0;
    double latSouth = 0.0;
    double latNorth = 0.0;
    int width = 0;
    int height = 0;
};

class EtopoTile {
public:
    bool LoadFromFile(const std::filesystem::path& path, std::string& error);
    void OverrideBoundsFromTileKey(int latSouthDeg, int lonWestDeg, int tileSpanDeg = 15);

    [[nodiscard]] bool IsValid() const { return !m_heights.empty() && m_width > 1 && m_height > 1; }
    [[nodiscard]] double SampleHeightMeters(double latDeg, double lonDeg) const;
    [[nodiscard]] double SampleHeightMetersDebug(double latDeg, double lonDeg, TileSampleDebug& debug) const;

    [[nodiscard]] int Width() const { return m_width; }
    [[nodiscard]] int Height() const { return m_height; }

    [[nodiscard]] double LonWest() const { return m_lonWest; }
    [[nodiscard]] double LonEast() const { return m_lonEast; }
    [[nodiscard]] double LatSouth() const { return m_latSouth; }
    [[nodiscard]] double LatNorth() const { return m_latNorth; }

private:
    [[nodiscard]] double SampleHeightMetersInternal(double latDeg, double lonDeg, TileSampleDebug* debug) const;
    [[nodiscard]] bool MapLatLonToPixel(
        double latDeg,
        double lonDeg,
        double& outX,
        double& outY,
        PixelMappingMode* outMode,
        double* outMappedLonDeg,
        double* outLonWrapAppliedDeg) const;

    int m_width = 0;
    int m_height = 0;
    double m_lonWest = 0.0;
    double m_lonEast = 0.0;
    double m_latSouth = 0.0;
    double m_latNorth = 0.0;
    bool m_hasGeoTransform = false;
    std::array<double, 6> m_geoTransform{};
    std::array<double, 6> m_invGeoTransform{};
    std::vector<float> m_heights;
};

} // namespace flight::terrain
