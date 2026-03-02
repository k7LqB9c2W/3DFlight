#include "terrain/terrain_system.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdio>

namespace flight::terrain {
namespace {

void NormalizeLon(double& lonDeg) {
    while (lonDeg >= 180.0) {
        lonDeg -= 360.0;
    }
    while (lonDeg < -180.0) {
        lonDeg += 360.0;
    }
}

} // namespace

std::string TileKey::ToCompactString() const {
    char latHem = (latSouthDeg >= 0) ? 'N' : 'S';
    char lonHem = (lonWestDeg >= 0) ? 'E' : 'W';
    char buffer[16]{};
    std::snprintf(buffer, sizeof(buffer), "%c%02d%c%03d", latHem, std::abs(latSouthDeg), lonHem, std::abs(lonWestDeg));
    return std::string(buffer);
}

std::string TileKey::ToFilename() const {
    return "ETOPO_2022_v1_15s_" + ToCompactString() + "_surface.tif";
}

std::size_t TileKeyHasher::operator()(const TileKey& key) const noexcept {
    const std::uint64_t a = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.latSouthDeg + 1024));
    const std::uint64_t b = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.lonWestDeg + 2048));
    return static_cast<std::size_t>((a << 32u) ^ b);
}

bool TerrainSystem::Initialize(const std::filesystem::path& tilesDirectory, std::size_t cacheCapacity, std::string& error) {
    if (!std::filesystem::exists(tilesDirectory)) {
        error = "Tiles directory does not exist: " + tilesDirectory.string();
        return false;
    }

    m_tilesDirectory = tilesDirectory;
    m_cacheCapacity = std::max<std::size_t>(cacheCapacity, 9);
    m_cache.clear();
    m_lru.clear();
    m_lruLookup.clear();
    return true;
}

TileKey TerrainSystem::KeyForLatLon(double latDeg, double lonDeg) {
    latDeg = std::clamp(latDeg, -90.0, 89.999999);
    NormalizeLon(lonDeg);

    const int tileLat = static_cast<int>(std::floor(latDeg / 15.0)) * 15;
    const int tileLon = static_cast<int>(std::floor(lonDeg / 15.0)) * 15;
    return TileKey{tileLat, tileLon};
}

double TerrainSystem::SampleHeightMeters(double latDeg, double lonDeg, bool* outTileLoaded) {
    const TileKey key = KeyForLatLon(latDeg, lonDeg);

    bool loaded = false;
    TilePtr tile = GetOrLoadTile(key, loaded);
    if (outTileLoaded != nullptr) {
        *outTileLoaded = loaded;
    }

    if (!tile) {
        return 0.0;
    }

    return tile->SampleHeightMeters(latDeg, lonDeg);
}

bool TerrainSystem::IsTileLoaded(const TileKey& key) const {
    return m_cache.find(key) != m_cache.end();
}

std::size_t TerrainSystem::LoadedTileCount() const {
    return m_cache.size();
}

TerrainSystem::TilePtr TerrainSystem::GetOrLoadTile(const TileKey& key, bool& outLoaded) {
    outLoaded = false;

    const auto found = m_cache.find(key);
    if (found != m_cache.end()) {
        TouchLru(key);
        outLoaded = true;
        return found->second;
    }

    const std::filesystem::path tilePath = ResolveTilePath(key);
    if (tilePath.empty() || !std::filesystem::exists(tilePath)) {
        return {};
    }

    auto tile = std::make_shared<EtopoTile>();
    std::string error;
    if (!tile->LoadFromFile(tilePath, error)) {
        return {};
    }

    m_cache.emplace(key, tile);
    m_lru.push_front(key);
    m_lruLookup[key] = m_lru.begin();
    EvictIfNeeded();

    outLoaded = true;
    return tile;
}

void TerrainSystem::TouchLru(const TileKey& key) {
    const auto found = m_lruLookup.find(key);
    if (found == m_lruLookup.end()) {
        m_lru.push_front(key);
        m_lruLookup[key] = m_lru.begin();
        return;
    }

    m_lru.splice(m_lru.begin(), m_lru, found->second);
    found->second = m_lru.begin();
}

void TerrainSystem::EvictIfNeeded() {
    while (m_cache.size() > m_cacheCapacity && !m_lru.empty()) {
        const TileKey tail = m_lru.back();
        m_lru.pop_back();
        m_lruLookup.erase(tail);
        m_cache.erase(tail);
    }
}

std::filesystem::path TerrainSystem::ResolveTilePath(const TileKey& key) const {
    const std::filesystem::path preferred = m_tilesDirectory / key.ToFilename();
    if (std::filesystem::exists(preferred)) {
        return preferred;
    }

    for (const auto& entry : std::filesystem::directory_iterator(m_tilesDirectory)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (entry.path().filename().string() == key.ToFilename()) {
            return entry.path();
        }
    }
    return {};
}

} // namespace flight::terrain
