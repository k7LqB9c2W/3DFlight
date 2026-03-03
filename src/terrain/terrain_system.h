#pragma once

#include <cstddef>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <list>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>
#include <unordered_map>

#include "terrain/etopo_tile.h"

namespace flight::terrain {

struct TileKey {
    int latSouthDeg = 0;
    int lonWestDeg = 0;

    [[nodiscard]] std::string ToCompactString() const;
    [[nodiscard]] std::string ToFilename() const;

    friend bool operator==(const TileKey& a, const TileKey& b) {
        return a.latSouthDeg == b.latSouthDeg && a.lonWestDeg == b.lonWestDeg;
    }
};

struct TileKeyHasher {
    std::size_t operator()(const TileKey& key) const noexcept;
};

struct TerrainSampleDebug {
    double latDeg = 0.0;
    double lonDeg = 0.0;
    TileKey key{};

    std::filesystem::path expectedTilePath;
    std::filesystem::path resolvedTilePath;
    bool tilePathFound = false;
    bool cacheHit = false;
    bool tileLoaded = false;
    bool tileLoadedThisCall = false;

    double sampledHeightMeters = 0.0;
    TileSampleDebug tileSample;
};

class TerrainSystem {
public:
    TerrainSystem() = default;
    ~TerrainSystem();
    TerrainSystem(const TerrainSystem&) = delete;
    TerrainSystem& operator=(const TerrainSystem&) = delete;

    bool Initialize(const std::filesystem::path& tilesDirectory, std::size_t cacheCapacity, std::string& error);
    void Shutdown();

    [[nodiscard]] static TileKey KeyForLatLon(double latDeg, double lonDeg);

    double SampleHeightMeters(double latDeg, double lonDeg, bool* outTileLoaded = nullptr);
    double SampleHeightMetersDebug(double latDeg, double lonDeg, TerrainSampleDebug& debug);
    double SampleHeightMetersCached(double latDeg, double lonDeg, bool* outTileLoaded = nullptr);
    double SampleHeightMetersDebugCached(double latDeg, double lonDeg, TerrainSampleDebug& debug);
    void PrefetchAround(double latDeg, double lonDeg, double headingDeg, double speedMps, int radiusTiles = 2);

    [[nodiscard]] bool IsTileLoaded(const TileKey& key) const;
    [[nodiscard]] std::size_t LoadedTileCount() const;
    [[nodiscard]] std::size_t CacheCapacity() const { return m_cacheCapacity; }
    [[nodiscard]] const std::filesystem::path& TilesDirectory() const { return m_tilesDirectory; }

private:
    using TilePtr = std::shared_ptr<EtopoTile>;

    void StartWorker();
    void StopWorker();
    void WorkerLoop();
    void QueuePrefetchKey(const TileKey& key);
    TilePtr GetOrLoadTile(const TileKey& key, bool& outLoaded);
    TilePtr GetLoadedTile(const TileKey& key, bool& outLoaded);
    void TouchLru(const TileKey& key);
    void EvictIfNeeded();
    [[nodiscard]] std::filesystem::path ResolveTilePath(const TileKey& key) const;

    std::filesystem::path m_tilesDirectory;
    std::size_t m_cacheCapacity = 9;

    std::unordered_map<TileKey, TilePtr, TileKeyHasher> m_cache;
    std::list<TileKey> m_lru;
    std::unordered_map<TileKey, std::list<TileKey>::iterator, TileKeyHasher> m_lruLookup;

    mutable std::mutex m_mutex;
    std::condition_variable m_prefetchCv;
    std::thread m_prefetchThread;
    std::deque<TileKey> m_prefetchQueue;
    std::unordered_set<TileKey, TileKeyHasher> m_prefetchQueuedSet;
    bool m_prefetchStop = false;
};

} // namespace flight::terrain
