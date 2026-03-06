#pragma once

#include <array>
#include <cstdint>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "satellite/satellite_streamer.h"

namespace flight::satellite {

struct WorldTileKey {
    int z = 0;
    int x = 0;
    int y = 0;
    friend bool operator==(const WorldTileKey& a, const WorldTileKey& b) {
        return a.z == b.z && a.x == b.x && a.y == b.y;
    }
};

struct WorldTileKeyHasher {
    size_t operator()(const WorldTileKey& key) const noexcept {
        const uint64_t a = static_cast<uint64_t>(static_cast<uint32_t>(key.z));
        const uint64_t b = static_cast<uint64_t>(static_cast<uint32_t>(key.x));
        const uint64_t c = static_cast<uint64_t>(static_cast<uint32_t>(key.y));
        return static_cast<size_t>((a << 48u) ^ (b << 24u) ^ c);
    }
};

struct WorldAtlasUpload {
    enum NeighborIndex : size_t {
        Left = 0,
        Right,
        Top,
        Bottom,
        TopLeft,
        TopRight,
        BottomLeft,
        BottomRight,
        NeighborCount
    };

    WorldTileKey key{};
    uint32_t atlasPageX = 0;
    uint32_t atlasPageY = 0;
    std::shared_ptr<const std::vector<uint8_t>> rgbaPixels;
    std::array<std::shared_ptr<const std::vector<uint8_t>>, NeighborCount> neighborRgbaPixels{};
};

struct WorldPageTableUpdate {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t key0 = 0;
    uint32_t key1 = 0;
    uint32_t value = 0;
};

struct WorldTileSystemStats {
    bool enabled = false;
    uint64_t frameId = 0;
    size_t visibleTileCount = 0;
    size_t residentTileCount = 0;
    size_t protectedTileCount = 0;
    size_t pendingUploadCount = 0;
    size_t pageTableOccupancy = 0;
    uint64_t streamerRequests = 0;
    uint64_t streamerCacheHits = 0;
    uint64_t streamerCacheMisses = 0;
    uint64_t atlasUploads = 0;
    uint64_t pageTableWrites = 0;
    uint64_t evictions = 0;
    uint64_t uploadSkipsNoPage = 0;
    uint64_t evictionSkipsSticky = 0;
    int activeZoomNear = 0;
    int activeZoomMid = 0;
    int activeZoomFar = 0;
    int activeZoomBand = 0;
    uint64_t zoomBandHoldFramesRemaining = 0;
    int governorLevel = 0;
    size_t governorRequestBudget = 0;
    size_t governorUploadBudget = 0;
    double governorFrameTimeMs = 0.0;
    size_t pageTableCapacity = 0;
    size_t pageTableTombstoneCount = 0;
    double pageTableLoadFactor = 0.0;
    double pageTableTombstoneRatio = 0.0;
    uint32_t shaderProbeBudget = 0;
    uint32_t maxProbeDistance = 0;
    double avgProbeDistance = 0.0;
    size_t visibleResidentCount = 0;
    size_t visibleResidentOverProbeBudget = 0;
};

class WorldTileSystem {
public:
    struct Config {
        int minZoom = 9;
        int maxZoom = 16;
        int atlasPagesX = 24;
        int atlasPagesY = 24;
        int pageTableWidth = 1024;
        int pageTableHeight = 1024;
        size_t maxVisibleTilesPerFrame = 1600;
        size_t maxRequestsPerFrame = 300;
        size_t maxUploadsPerFrame = 12;
        size_t minProtectedRequestsPerFrame = 48;
        size_t minProtectedUploadsPerFrame = 3;
        size_t nearResidentReservePages = 96;
        uint64_t requestCooldownFrames = 2;
        uint64_t staleNonResidentFrames = 240;
        uint64_t zoomSwitchHoldFrames = 45;
        double altitudeBandHysteresisMeters = 180.0;
        uint64_t governorRecoveryHoldFrames = 45;
        uint32_t shaderProbeBudget = 8;
        uint64_t residentStickinessFrames = 180;
    };

    struct ViewState {
        double latDeg = 0.0;
        double lonDeg = 0.0;
        double altitudeMeters = 0.0;
        double headingDeg = 0.0;
    };

    WorldTileSystem() = default;

    bool Initialize(const Config& config, std::string& error);
    void Reset();
    void SetEnabled(bool enabled) { m_enabled = enabled; }
    [[nodiscard]] bool IsEnabled() const { return m_enabled; }
    void SetFrameTimeMs(double frameTimeMs);

    void Tick(const ViewState& view, SatelliteStreamer& streamer);
    void ConsumeGpuUpdates(std::vector<WorldAtlasUpload>& outAtlas, std::vector<WorldPageTableUpdate>& outPageTable);
    [[nodiscard]] WorldTileSystemStats GetStats() const;

private:
    struct TileState {
        bool visibleThisFrame = false;
        bool protectedNear = false;
        bool resident = false;
        bool pendingUpload = false;
        int atlasPageIndex = -1;
        uint32_t pageTableProbeDistance = 0;
        uint64_t lastTouchedFrame = 0;
        uint64_t lastRequestFrame = 0;
        std::shared_ptr<const std::vector<uint8_t>> decodedPixels;
        std::list<WorldTileKey>::iterator lruIt{};
        bool inLru = false;
    };

    struct VisibleTileCandidate {
        WorldTileKey key{};
        bool nearProtected = false;
        bool highPriority = false;
        float priority = 0.0f;
    };

    static int ComputeDesiredZoomBand(double altitudeMeters);
    static std::array<int, 3> ZoomTripletForBand(int bandIndex);
    static uint32_t HashTileKeyForPageTable(const WorldTileKey& key);
    [[nodiscard]] std::vector<VisibleTileCandidate> ComputeVisibleTiles(const ViewState& view) const;
    void UpdateZoomBandWithHysteresis(double altitudeMeters);
    void UpdateFrameTimeGovernor();
    void TouchResident(const WorldTileKey& key, TileState& state);
    std::optional<int> AllocateAtlasPage(const WorldTileKey& forKey, bool protectedRequest);
    void EvictResident(const WorldTileKey& key, TileState& state);
    std::optional<uint32_t> AssignPageTableSlot(const WorldTileKey& key);
    void ReleasePageTableSlot(const WorldTileKey& key);
    static uint32_t PackPageTableKey0(const WorldTileKey& key);
    static uint32_t PackPageTableKey1(const WorldTileKey& key);
    static uint32_t PackPageTableValue(const WorldTileKey& key, int atlasPageIndex);
    void GarbageCollect();

    Config m_config{};
    bool m_initialized = false;
    bool m_enabled = false;
    uint64_t m_frameId = 0;
    std::array<int, 3> m_activeZooms{13, 12, 10};
    int m_activeZoomBand = 3;
    uint64_t m_lastZoomBandSwitchFrame = 0;
    double m_lastFrameTimeMs = 16.67;
    int m_governorLevel = 0;
    uint64_t m_lastGovernorChangeFrame = 0;
    size_t m_effectiveRequestBudget = 0;
    size_t m_effectiveUploadBudget = 0;

    std::vector<std::optional<WorldTileKey>> m_pageToKey;
    std::vector<int> m_freeAtlasPages;

    std::vector<std::optional<WorldTileKey>> m_pageTableSlots;
    std::vector<uint8_t> m_pageTableEverUsed;
    std::unordered_map<WorldTileKey, uint32_t, WorldTileKeyHasher> m_keyToPageTableSlot;

    std::unordered_map<WorldTileKey, TileState, WorldTileKeyHasher> m_tiles;
    std::list<WorldTileKey> m_residentLru;

    std::vector<WorldAtlasUpload> m_pendingAtlasUploads;
    std::vector<WorldPageTableUpdate> m_pendingPageUpdates;

    size_t m_visibleTileCount = 0;
    uint64_t m_streamerRequests = 0;
    uint64_t m_streamerCacheHits = 0;
    uint64_t m_streamerCacheMisses = 0;
    uint64_t m_atlasUploads = 0;
    uint64_t m_pageTableWrites = 0;
    uint64_t m_evictions = 0;
    uint64_t m_uploadSkipsNoPage = 0;
    uint64_t m_evictionSkipsSticky = 0;
    size_t m_pageTableTombstoneCount = 0;
    uint32_t m_debugMaxProbeDistance = 0;
    double m_debugAvgProbeDistance = 0.0;
    size_t m_debugVisibleResidentCount = 0;
    size_t m_debugVisibleResidentOverProbeBudget = 0;
};

} // namespace flight::satellite
