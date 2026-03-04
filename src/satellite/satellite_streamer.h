#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <DirectXMath.h>

namespace flight::satellite {

struct RingTexture {
    std::vector<uint8_t> rgbaPixels;
    uint32_t width = 0;
    uint32_t height = 0;
    // Unwrapped lon bounds in degrees [lonWest, lonEast], lat bounds [latSouth, latNorth].
    DirectX::XMFLOAT4 boundsLonLat{0.0f, 0.0f, 0.0f, 0.0f};
    int zoom = 0;
    bool valid = false;
};

struct StreamStats {
    uint64_t diskHits = 0;
    uint64_t networkHits = 0;
    uint64_t networkFailures = 0;
    uint64_t decodeFailures = 0;
    size_t memoryTileCount = 0;
    size_t queuedRequests = 0;
    std::array<int, 3> activeZooms{0, 0, 0};
};

class SatelliteStreamer {
public:
    struct Config {
        std::string urlTemplate = "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2024_3857/default/GoogleMapsCompatible/{z}/{y}/{x}.jpg";
        std::filesystem::path diskCacheDirectory = std::filesystem::path("cache") / "satellite" / "s2cloudless-2024_3857";
        size_t maxMemoryTiles = 1024;
        int workerCount = 4;
        int nearTextureSize = 1024;
        int midTextureSize = 1024;
        int farTextureSize = 1024;
    };

    SatelliteStreamer() = default;
    ~SatelliteStreamer();
    SatelliteStreamer(const SatelliteStreamer&) = delete;
    SatelliteStreamer& operator=(const SatelliteStreamer&) = delete;

    bool Initialize(const Config& config, std::string& error);
    void Shutdown();

    void PrefetchForView(double latDeg, double lonDeg, double altitudeMeters);
    bool ComposeLodRings(
        double latDeg,
        double lonDeg,
        double altitudeMeters,
        bool force,
        std::array<RingTexture, 3>& outRings,
        std::string& error);
    bool QueueTileRequest(int zoom, int tileX, int tileY, bool highPriority);
    bool TryGetCachedTileRgba(
        int zoom,
        int tileX,
        int tileY,
        std::shared_ptr<const std::vector<uint8_t>>& outPixels);

    [[nodiscard]] StreamStats GetStats() const;

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

} // namespace flight::satellite
