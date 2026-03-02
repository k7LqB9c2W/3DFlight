#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace flight::terrain {

class EtopoTile {
public:
    bool LoadFromFile(const std::filesystem::path& path, std::string& error);

    [[nodiscard]] bool IsValid() const { return !m_heights.empty() && m_width > 1 && m_height > 1; }
    [[nodiscard]] double SampleHeightMeters(double latDeg, double lonDeg) const;

    [[nodiscard]] int Width() const { return m_width; }
    [[nodiscard]] int Height() const { return m_height; }

    [[nodiscard]] double LonWest() const { return m_lonWest; }
    [[nodiscard]] double LonEast() const { return m_lonEast; }
    [[nodiscard]] double LatSouth() const { return m_latSouth; }
    [[nodiscard]] double LatNorth() const { return m_latNorth; }

private:
    int m_width = 0;
    int m_height = 0;
    double m_lonWest = 0.0;
    double m_lonEast = 0.0;
    double m_latSouth = 0.0;
    double m_latNorth = 0.0;
    std::vector<float> m_heights;
};

} // namespace flight::terrain
