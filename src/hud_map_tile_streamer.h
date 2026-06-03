#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace flight::hud {

struct HudMapTileStats {
    uint64_t diskHits = 0;
    uint64_t networkHits = 0;
    uint64_t networkFailures = 0;
    uint64_t decodeFailures = 0;
    size_t memoryTileCount = 0;
    size_t queuedRequests = 0;
};

class HudMapTileStreamer {
public:
    struct Config {
        std::string urlTemplate = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
        std::filesystem::path diskCacheDirectory = std::filesystem::path("cache") / "hud_map" / "osm";
        size_t maxMemoryTiles = 1024;
        int workerCount = 2;
    };

    HudMapTileStreamer() = default;
    ~HudMapTileStreamer();
    HudMapTileStreamer(const HudMapTileStreamer&) = delete;
    HudMapTileStreamer& operator=(const HudMapTileStreamer&) = delete;

    bool Initialize(const Config& config, std::string& error);
    void Shutdown();

    bool QueueTileRequest(int zoom, int tileX, int tileY, bool highPriority);
    bool TryGetCachedTileRgba(
        int zoom,
        int tileX,
        int tileY,
        std::shared_ptr<const std::vector<uint8_t>>& outPixels);

    [[nodiscard]] HudMapTileStats GetStats() const;

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

} // namespace flight::hud
