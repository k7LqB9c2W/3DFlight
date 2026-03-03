#include "terrain/terrain_system.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sim.h"

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

int NormalizeTileLon(int lonDeg) {
    while (lonDeg >= 180) {
        lonDeg -= 360;
    }
    while (lonDeg < -180) {
        lonDeg += 360;
    }
    return lonDeg;
}

int ClampTileLatNorthKey(int latKeyDeg) {
    return std::clamp(latKeyDeg, -75, 90);
}

void OffsetLatLonByHeading(double latDeg, double lonDeg, double headingDeg, double distanceMeters, double& outLatDeg, double& outLonDeg) {
    const double lat1 = DegToRad(latDeg);
    const double lon1 = DegToRad(lonDeg);
    const double bearing = DegToRad(headingDeg);
    const double delta = distanceMeters / kEarthRadiusMeters;

    const double sinLat1 = std::sin(lat1);
    const double cosLat1 = std::cos(lat1);
    const double sinDelta = std::sin(delta);
    const double cosDelta = std::cos(delta);

    const double sinLat2 = std::clamp(sinLat1 * cosDelta + cosLat1 * sinDelta * std::cos(bearing), -1.0, 1.0);
    const double lat2 = std::asin(sinLat2);
    const double y = std::sin(bearing) * sinDelta * cosLat1;
    const double x = cosDelta - sinLat1 * sinLat2;
    const double lon2 = lon1 + std::atan2(y, x);

    outLatDeg = std::clamp(RadToDeg(lat2), -89.999, 89.999);
    outLonDeg = RadToDeg(lon2);
    NormalizeLon(outLonDeg);
}

} // namespace

TerrainSystem::~TerrainSystem() {
    Shutdown();
}

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

    Shutdown();

    {
        std::scoped_lock<std::mutex> lock(m_mutex);
        m_tilesDirectory = tilesDirectory;
        // Keep at least 5x5 terrain tile residency.
        m_cacheCapacity = std::max<std::size_t>(cacheCapacity, 25);
        m_cache.clear();
        m_lru.clear();
        m_lruLookup.clear();
        m_prefetchQueue.clear();
        m_prefetchQueuedSet.clear();
        m_prefetchStop = false;
    }

    StartWorker();
    return true;
}

void TerrainSystem::Shutdown() {
    StopWorker();
    std::scoped_lock<std::mutex> lock(m_mutex);
    m_cache.clear();
    m_lru.clear();
    m_lruLookup.clear();
    m_prefetchQueue.clear();
    m_prefetchQueuedSet.clear();
}

void TerrainSystem::StartWorker() {
    std::scoped_lock<std::mutex> lock(m_mutex);
    if (m_prefetchThread.joinable()) {
        return;
    }
    m_prefetchStop = false;
    m_prefetchThread = std::thread([this]() { WorkerLoop(); });
}

void TerrainSystem::StopWorker() {
    {
        std::scoped_lock<std::mutex> lock(m_mutex);
        m_prefetchStop = true;
    }
    m_prefetchCv.notify_all();
    if (m_prefetchThread.joinable()) {
        m_prefetchThread.join();
    }
}

void TerrainSystem::WorkerLoop() {
    while (true) {
        TileKey key{};
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_prefetchCv.wait(lock, [this]() { return m_prefetchStop || !m_prefetchQueue.empty(); });
            if (m_prefetchStop && m_prefetchQueue.empty()) {
                break;
            }
            key = m_prefetchQueue.front();
            m_prefetchQueue.pop_front();
            m_prefetchQueuedSet.erase(key);
        }

        bool loaded = false;
        (void)GetOrLoadTile(key, loaded);
    }
}

TileKey TerrainSystem::KeyForLatLon(double latDeg, double lonDeg) {
    latDeg = std::clamp(latDeg, -89.999999, 89.999999);
    NormalizeLon(lonDeg);

    // ETOPO 15s filenames are keyed by the tile's northern latitude edge.
    // Example: N45W135 covers lat [30,45] and lon [-135,-120].
    constexpr double kBinEpsilon = 1e-9;
    const int tileLat = static_cast<int>(std::floor((latDeg - kBinEpsilon) / 15.0)) * 15 + 15;
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

double TerrainSystem::SampleHeightMetersCached(double latDeg, double lonDeg, bool* outTileLoaded) {
    const TileKey key = KeyForLatLon(latDeg, lonDeg);

    bool loaded = false;
    TilePtr tile = GetLoadedTile(key, loaded);
    if (outTileLoaded != nullptr) {
        *outTileLoaded = loaded;
    }
    if (!tile) {
        return 0.0;
    }
    return tile->SampleHeightMeters(latDeg, lonDeg);
}

double TerrainSystem::SampleHeightMetersDebug(double latDeg, double lonDeg, TerrainSampleDebug& debug) {
    debug = {};
    debug.latDeg = latDeg;
    debug.lonDeg = lonDeg;
    debug.key = KeyForLatLon(latDeg, lonDeg);
    debug.expectedTilePath = m_tilesDirectory / debug.key.ToFilename();
    debug.resolvedTilePath = ResolveTilePath(debug.key);
    debug.tilePathFound = !debug.resolvedTilePath.empty();
    debug.cacheHit = IsTileLoaded(debug.key);

    bool loaded = false;
    TilePtr tile = GetOrLoadTile(debug.key, loaded);
    debug.tileLoaded = loaded && (tile != nullptr);
    debug.tileLoadedThisCall = debug.tileLoaded && !debug.cacheHit;
    if (!tile) {
        debug.sampledHeightMeters = 0.0;
        return 0.0;
    }

    debug.sampledHeightMeters = tile->SampleHeightMetersDebug(latDeg, lonDeg, debug.tileSample);
    return debug.sampledHeightMeters;
}

double TerrainSystem::SampleHeightMetersDebugCached(double latDeg, double lonDeg, TerrainSampleDebug& debug) {
    debug = {};
    debug.latDeg = latDeg;
    debug.lonDeg = lonDeg;
    debug.key = KeyForLatLon(latDeg, lonDeg);
    debug.expectedTilePath = m_tilesDirectory / debug.key.ToFilename();
    debug.resolvedTilePath = ResolveTilePath(debug.key);
    debug.tilePathFound = !debug.resolvedTilePath.empty();

    bool loaded = false;
    TilePtr tile = GetLoadedTile(debug.key, loaded);
    debug.cacheHit = loaded;
    debug.tileLoaded = loaded && (tile != nullptr);
    debug.tileLoadedThisCall = false;
    if (!tile) {
        debug.sampledHeightMeters = 0.0;
        return 0.0;
    }

    debug.sampledHeightMeters = tile->SampleHeightMetersDebug(latDeg, lonDeg, debug.tileSample);
    return debug.sampledHeightMeters;
}

void TerrainSystem::PrefetchAround(double latDeg, double lonDeg, double headingDeg, double speedMps, int radiusTiles) {
    radiusTiles = std::clamp(radiusTiles, 1, 4);

    std::unordered_set<TileKey, TileKeyHasher> uniqueKeys;
    auto addNeighborhood = [&](const TileKey& center, int radius) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                TileKey key{};
                key.latSouthDeg = ClampTileLatNorthKey(center.latSouthDeg + dy * 15);
                key.lonWestDeg = NormalizeTileLon(center.lonWestDeg + dx * 15);
                uniqueKeys.insert(key);
            }
        }
    };

    addNeighborhood(KeyForLatLon(latDeg, lonDeg), radiusTiles);

    const int lookAheadSteps = std::clamp(static_cast<int>(speedMps / 140.0) + 1, 1, 4);
    for (int i = 1; i <= lookAheadSteps; ++i) {
        const double lookAheadMeters = static_cast<double>(i) * 180000.0;
        double predLatDeg = latDeg;
        double predLonDeg = lonDeg;
        OffsetLatLonByHeading(latDeg, lonDeg, headingDeg, lookAheadMeters, predLatDeg, predLonDeg);
        addNeighborhood(KeyForLatLon(predLatDeg, predLonDeg), 1);
    }

    for (const TileKey& key : uniqueKeys) {
        QueuePrefetchKey(key);
    }
}

void TerrainSystem::QueuePrefetchKey(const TileKey& key) {
    std::scoped_lock<std::mutex> lock(m_mutex);
    if (m_cache.find(key) != m_cache.end()) {
        return;
    }
    if (m_prefetchQueuedSet.find(key) != m_prefetchQueuedSet.end()) {
        return;
    }
    m_prefetchQueue.push_back(key);
    m_prefetchQueuedSet.insert(key);
    m_prefetchCv.notify_one();
}

bool TerrainSystem::IsTileLoaded(const TileKey& key) const {
    std::scoped_lock<std::mutex> lock(m_mutex);
    return m_cache.find(key) != m_cache.end();
}

std::size_t TerrainSystem::LoadedTileCount() const {
    std::scoped_lock<std::mutex> lock(m_mutex);
    return m_cache.size();
}

TerrainSystem::TilePtr TerrainSystem::GetLoadedTile(const TileKey& key, bool& outLoaded) {
    outLoaded = false;

    std::scoped_lock<std::mutex> lock(m_mutex);
    const auto found = m_cache.find(key);
    if (found == m_cache.end()) {
        return {};
    }
    TouchLru(key);
    outLoaded = true;
    return found->second;
}

TerrainSystem::TilePtr TerrainSystem::GetOrLoadTile(const TileKey& key, bool& outLoaded) {
    outLoaded = false;

    {
        std::scoped_lock<std::mutex> lock(m_mutex);
        const auto found = m_cache.find(key);
        if (found != m_cache.end()) {
            TouchLru(key);
            outLoaded = true;
            return found->second;
        }
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
    // ETOPO filenames are keyed by the north-west tag (NxxWyyy), so the actual
    // bounds are lat [tag-15, tag] and lon [tag, tag+15].
    tile->OverrideBoundsFromTileKey(key.latSouthDeg - 15, key.lonWestDeg, 15);

    std::scoped_lock<std::mutex> lock(m_mutex);
    const auto foundAfterLoad = m_cache.find(key);
    if (foundAfterLoad != m_cache.end()) {
        TouchLru(key);
        outLoaded = true;
        return foundAfterLoad->second;
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
