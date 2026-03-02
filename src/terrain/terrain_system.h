#pragma once

#include <cstddef>
#include <filesystem>
#include <list>
#include <memory>
#include <string>
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

class TerrainSystem {
public:
    bool Initialize(const std::filesystem::path& tilesDirectory, std::size_t cacheCapacity, std::string& error);

    [[nodiscard]] static TileKey KeyForLatLon(double latDeg, double lonDeg);

    double SampleHeightMeters(double latDeg, double lonDeg, bool* outTileLoaded = nullptr);

    [[nodiscard]] bool IsTileLoaded(const TileKey& key) const;
    [[nodiscard]] std::size_t LoadedTileCount() const;
    [[nodiscard]] std::size_t CacheCapacity() const { return m_cacheCapacity; }
    [[nodiscard]] const std::filesystem::path& TilesDirectory() const { return m_tilesDirectory; }

private:
    using TilePtr = std::shared_ptr<EtopoTile>;

    TilePtr GetOrLoadTile(const TileKey& key, bool& outLoaded);
    void TouchLru(const TileKey& key);
    void EvictIfNeeded();
    [[nodiscard]] std::filesystem::path ResolveTilePath(const TileKey& key) const;

    std::filesystem::path m_tilesDirectory;
    std::size_t m_cacheCapacity = 9;

    std::unordered_map<TileKey, TilePtr, TileKeyHasher> m_cache;
    std::list<TileKey> m_lru;
    std::unordered_map<TileKey, std::list<TileKey>::iterator, TileKeyHasher> m_lruLookup;
};

} // namespace flight::terrain
