#include "satellite/world_tile_system.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace flight::satellite {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr uint32_t kPageTableTombstoneValue = 0x40000000u;
constexpr std::array<double, 3> kBandBoundariesMeters{500.0, 2000.0, 6000.0};
constexpr std::array<double, 3> kBandLookaheadFractions{0.12, 0.22, 0.34};
constexpr std::array<double, 3> kBandLookaheadSeconds{1.5, 3.0, 5.0};
constexpr std::array<double, 3> kBandLookaheadCapFractions{0.20, 0.35, 0.48};

constexpr std::array<std::array<int, 3>, 4> kBandZoomTriplets{{
    {{16, 15, 13}},
    {{15, 14, 12}},
    {{14, 13, 11}},
    {{13, 12, 10}},
}};

constexpr std::array<std::array<double, 3>, 4> kBandRadiusKm{{
    {{10.0, 34.0, 120.0}},
    {{14.0, 48.0, 170.0}},
    {{20.0, 72.0, 250.0}},
    {{28.0, 100.0, 340.0}},
}};
constexpr float kNearProtectionDistNorm = 0.38f;

double WrapLonDeg(double lonDeg) {
    while (lonDeg >= 180.0) {
        lonDeg -= 360.0;
    }
    while (lonDeg < -180.0) {
        lonDeg += 360.0;
    }
    return lonDeg;
}

double ClampLatDeg(double latDeg) {
    return std::clamp(latDeg, -85.05112878, 85.05112878);
}

double LonToTileXUnwrapped(double lonDeg, int z) {
    const double n = static_cast<double>(1u << z);
    return (lonDeg + 180.0) / 360.0 * n;
}

double LatToTileY(double latDeg, int z) {
    const double n = static_cast<double>(1u << z);
    const double latRad = ClampLatDeg(latDeg) * (kPi / 180.0);
    const double y = (1.0 - std::log(std::tan(latRad) + 1.0 / std::cos(latRad)) / kPi) * 0.5 * n;
    return std::clamp(y, 0.0, n - 1.0e-6);
}

int WrapTileX(int x, int n) {
    int r = x % n;
    if (r < 0) {
        r += n;
    }
    return r;
}

void OffsetLatLonByHeading(double latDeg, double lonDeg, double headingDeg, double distanceMeters, double& outLatDeg, double& outLonDeg) {
    const double lat1 = ClampLatDeg(latDeg) * (kPi / 180.0);
    const double lon1 = WrapLonDeg(lonDeg) * (kPi / 180.0);
    const double bearing = headingDeg * (kPi / 180.0);
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

    outLatDeg = ClampLatDeg(lat2 * (180.0 / kPi));
    outLonDeg = WrapLonDeg(lon2 * (180.0 / kPi));
}

bool HasDecodedTilePixels(const std::shared_ptr<const std::vector<uint8_t>>& pixels) {
    return pixels && pixels->size() >= (256u * 256u * 4u);
}

WorldTileKey OffsetWorldTileKey(const WorldTileKey& key, int dx, int dy) {
    const int n = 1 << key.z;
    WorldTileKey shifted = key;
    shifted.x = WrapTileX(key.x + dx, n);
    shifted.y = std::clamp(key.y + dy, 0, n - 1);
    return shifted;
}

void PopulateUploadNeighborPixels(
    const WorldTileKey& key,
    SatelliteStreamer& streamer,
    WorldAtlasUpload& upload) {
    static constexpr std::array<std::pair<int, int>, WorldAtlasUpload::NeighborCount> kNeighborOffsets{{
        {-1, 0},
        {1, 0},
        {0, -1},
        {0, 1},
        {-1, -1},
        {1, -1},
        {-1, 1},
        {1, 1},
    }};

    for (size_t i = 0; i < kNeighborOffsets.size(); ++i) {
        const auto [dx, dy] = kNeighborOffsets[i];
        const WorldTileKey neighborKey = OffsetWorldTileKey(key, dx, dy);
        std::shared_ptr<const std::vector<uint8_t>> neighborPixels;
        if (streamer.TryGetCachedTileRgba(neighborKey.z, neighborKey.x, neighborKey.y, neighborPixels) &&
            HasDecodedTilePixels(neighborPixels)) {
            upload.neighborRgbaPixels[i] = std::move(neighborPixels);
        }
    }
}

double ApproxKmPerTile(double latDeg, int zoom) {
    const double n = static_cast<double>(1u << zoom);
    const double kmPerTileEq = 40075.016686 / n;
    const double cosLat = std::max(0.18, std::cos(ClampLatDeg(latDeg) * (kPi / 180.0)));
    return kmPerTileEq * cosLat;
}

double HeadingForwardComponentTiles(double dx, double dy, double headingDeg) {
    const double headingRad = headingDeg * (kPi / 180.0);
    const double east = std::sin(headingRad);
    const double north = std::cos(headingRad);
    return dx * east + (-dy) * north;
}

} // namespace

bool WorldTileSystem::Initialize(const Config& config, std::string& error) {
    error.clear();
    Reset();

    Config cfg = config;
    cfg.minZoom = std::clamp(cfg.minZoom, 0, 20);
    cfg.maxZoom = std::clamp(cfg.maxZoom, cfg.minZoom, 22);
    cfg.atlasPagesX = std::clamp(cfg.atlasPagesX, 4, 64);
    cfg.atlasPagesY = std::clamp(cfg.atlasPagesY, 4, 64);
    cfg.pageTableWidth = std::clamp(cfg.pageTableWidth, 64, 4096);
    cfg.pageTableHeight = std::clamp(cfg.pageTableHeight, 64, 4096);
    cfg.maxVisibleTilesPerFrame = std::clamp<size_t>(cfg.maxVisibleTilesPerFrame, 64, 10000);
    cfg.maxRequestsPerFrame = std::clamp<size_t>(cfg.maxRequestsPerFrame, 16, 2000);
    cfg.maxUploadsPerFrame = std::clamp<size_t>(cfg.maxUploadsPerFrame, 1, 256);
    cfg.minProtectedRequestsPerFrame = std::clamp<size_t>(cfg.minProtectedRequestsPerFrame, 8, cfg.maxRequestsPerFrame);
    cfg.minProtectedUploadsPerFrame = std::clamp<size_t>(cfg.minProtectedUploadsPerFrame, 1, cfg.maxUploadsPerFrame);
    cfg.requestCooldownFrames = std::clamp<uint64_t>(cfg.requestCooldownFrames, 1, 60);
    cfg.staleNonResidentFrames = std::clamp<uint64_t>(cfg.staleNonResidentFrames, 30, 2000);
    cfg.zoomSwitchHoldFrames = std::clamp<uint64_t>(cfg.zoomSwitchHoldFrames, 5, 300);
    cfg.altitudeBandHysteresisMeters = std::clamp(cfg.altitudeBandHysteresisMeters, 20.0, 2000.0);
    cfg.governorRecoveryHoldFrames = std::clamp<uint64_t>(cfg.governorRecoveryHoldFrames, 15, 300);
    cfg.shaderProbeBudget = std::clamp<uint32_t>(cfg.shaderProbeBudget, 1u, 16u);
    cfg.residentStickinessFrames = std::clamp<uint64_t>(cfg.residentStickinessFrames, 0, 900);

    const size_t atlasPageCount = static_cast<size_t>(cfg.atlasPagesX) * static_cast<size_t>(cfg.atlasPagesY);
    if (atlasPageCount == 0) {
        error = "Invalid world tile atlas page count";
        return false;
    }
    // Prevent permanent over-subscription (visible set far larger than resident atlas).
    const size_t visibleCapFromAtlas = std::max<size_t>(64, (atlasPageCount * 3u) / 2u);
    cfg.maxVisibleTilesPerFrame = std::min(cfg.maxVisibleTilesPerFrame, visibleCapFromAtlas);
    cfg.nearResidentReservePages = std::clamp<size_t>(
        cfg.nearResidentReservePages,
        std::min<size_t>(8, atlasPageCount),
        std::max<size_t>(8, atlasPageCount - 1u));
    m_config = cfg;

    m_pageToKey.assign(atlasPageCount, std::nullopt);
    m_freeAtlasPages.reserve(atlasPageCount);
    for (int i = static_cast<int>(atlasPageCount) - 1; i >= 0; --i) {
        m_freeAtlasPages.push_back(i);
    }

    const size_t pageTableSlots = static_cast<size_t>(cfg.pageTableWidth) * static_cast<size_t>(cfg.pageTableHeight);
    m_pageTableSlots.assign(pageTableSlots, std::nullopt);
    m_pageTableEverUsed.assign(pageTableSlots, 0u);
    m_keyToPageTableSlot.clear();

    m_activeZoomBand = 3;
    m_activeZooms = ZoomTripletForBand(m_activeZoomBand);
    for (int& z : m_activeZooms) {
        z = std::clamp(z, m_config.minZoom, m_config.maxZoom);
    }
    m_effectiveRequestBudget = m_config.maxRequestsPerFrame;
    m_effectiveUploadBudget = m_config.maxUploadsPerFrame;

    m_initialized = true;
    m_enabled = true;
    return true;
}

void WorldTileSystem::Reset() {
    m_initialized = false;
    m_enabled = false;
    m_frameId = 0;
    m_activeZooms = {13, 12, 10};
    m_activeZoomBand = 3;
    m_lastZoomBandSwitchFrame = 0;
    m_lastFrameTimeMs = 16.67;
    m_governorLevel = 0;
    m_lastGovernorChangeFrame = 0;
    m_effectiveRequestBudget = 0;
    m_effectiveUploadBudget = 0;
    m_pageToKey.clear();
    m_freeAtlasPages.clear();
    m_pageTableSlots.clear();
    m_pageTableEverUsed.clear();
    m_keyToPageTableSlot.clear();
    m_tiles.clear();
    m_residentLru.clear();
    m_pendingAtlasUploads.clear();
    m_pendingPageUpdates.clear();
    m_visibleTileCount = 0;
    m_streamerRequests = 0;
    m_streamerCacheHits = 0;
    m_streamerCacheMisses = 0;
    m_atlasUploads = 0;
    m_pageTableWrites = 0;
    m_evictions = 0;
    m_uploadSkipsNoPage = 0;
    m_evictionSkipsSticky = 0;
    m_pageTableTombstoneCount = 0;
    m_debugMaxProbeDistance = 0;
    m_debugAvgProbeDistance = 0.0;
    m_debugVisibleResidentCount = 0;
    m_debugVisibleResidentOverProbeBudget = 0;
}

void WorldTileSystem::SetFrameTimeMs(double frameTimeMs) {
    m_lastFrameTimeMs = std::clamp(frameTimeMs, 5.0, 80.0);
}

int WorldTileSystem::ComputeDesiredZoomBand(double altitudeMeters) {
    if (altitudeMeters < kBandBoundariesMeters[0]) {
        return 0;
    }
    if (altitudeMeters < kBandBoundariesMeters[1]) {
        return 1;
    }
    if (altitudeMeters < kBandBoundariesMeters[2]) {
        return 2;
    }
    return 3;
}

std::array<int, 3> WorldTileSystem::ZoomTripletForBand(int bandIndex) {
    const int idx = std::clamp(bandIndex, 0, static_cast<int>(kBandZoomTriplets.size() - 1));
    return kBandZoomTriplets[static_cast<size_t>(idx)];
}

uint32_t WorldTileSystem::HashTileKeyForPageTable(const WorldTileKey& key) {
    uint32_t h = 2166136261u; // FNV-1a base
    auto mix = [&h](uint32_t v) {
        h ^= v;
        h *= 16777619u;
    };
    mix(static_cast<uint32_t>(key.z));
    mix(static_cast<uint32_t>(key.x));
    mix(static_cast<uint32_t>(key.y));
    return h;
}

void WorldTileSystem::UpdateZoomBandWithHysteresis(double altitudeMeters) {
    if (!m_initialized) {
        return;
    }

    const int desiredBand = ComputeDesiredZoomBand(altitudeMeters);
    if (desiredBand == m_activeZoomBand) {
        return;
    }

    if ((m_frameId - m_lastZoomBandSwitchFrame) < m_config.zoomSwitchHoldFrames) {
        return;
    }

    const double hyst = m_config.altitudeBandHysteresisMeters;
    bool switched = false;

    if (desiredBand > m_activeZoomBand) {
        const int nextBand = std::min(3, m_activeZoomBand + 1);
        const int boundaryIdx = std::clamp(m_activeZoomBand, 0, 2);
        const double threshold = kBandBoundariesMeters[static_cast<size_t>(boundaryIdx)] + hyst * (1.0 + 0.35 * boundaryIdx);
        if (altitudeMeters >= threshold) {
            m_activeZoomBand = nextBand;
            switched = true;
        }
    } else {
        const int nextBand = std::max(0, m_activeZoomBand - 1);
        const int boundaryIdx = std::clamp(nextBand, 0, 2);
        const double threshold = kBandBoundariesMeters[static_cast<size_t>(boundaryIdx)] - hyst * (1.0 + 0.35 * boundaryIdx);
        if (altitudeMeters <= threshold) {
            m_activeZoomBand = nextBand;
            switched = true;
        }
    }

    if (switched) {
        m_lastZoomBandSwitchFrame = m_frameId;
        m_activeZooms = ZoomTripletForBand(m_activeZoomBand);
        for (int& z : m_activeZooms) {
            z = std::clamp(z, m_config.minZoom, m_config.maxZoom);
        }
    }
}

void WorldTileSystem::UpdateFrameTimeGovernor() {
    const double frameMs = std::clamp(m_lastFrameTimeMs, 5.0, 80.0);

    int targetLevel = 0;
    if (frameMs > 40.0) {
        targetLevel = 3;
    } else if (frameMs > 32.0) {
        targetLevel = 2;
    } else if (frameMs > 24.0) {
        targetLevel = 1;
    }

    if (targetLevel > m_governorLevel) {
        m_governorLevel = targetLevel;
        m_lastGovernorChangeFrame = m_frameId;
    } else if (targetLevel < m_governorLevel) {
        const bool canRecover = (m_frameId - m_lastGovernorChangeFrame) >= m_config.governorRecoveryHoldFrames;
        if (canRecover) {
            bool allowStepDown = false;
            if (m_governorLevel == 3) {
                allowStepDown = frameMs < 36.0;
            } else if (m_governorLevel == 2) {
                allowStepDown = frameMs < 29.0;
            } else if (m_governorLevel == 1) {
                allowStepDown = frameMs < 22.0;
            }
            if (allowStepDown) {
                m_governorLevel -= 1;
                m_lastGovernorChangeFrame = m_frameId;
            }
        }
    }

    const std::array<float, 4> requestScale{1.0f, 0.84f, 0.68f, 0.52f};
    const std::array<float, 4> uploadScale{1.0f, 0.88f, 0.72f, 0.56f};

    const float reqMul = requestScale[static_cast<size_t>(std::clamp(m_governorLevel, 0, 3))];
    const float upMul = uploadScale[static_cast<size_t>(std::clamp(m_governorLevel, 0, 3))];

    m_effectiveRequestBudget = std::clamp<size_t>(
        std::max<size_t>(
            m_config.minProtectedRequestsPerFrame,
            static_cast<size_t>(std::llround(static_cast<double>(m_config.maxRequestsPerFrame) * reqMul))),
        m_config.minProtectedRequestsPerFrame,
        m_config.maxRequestsPerFrame);
    m_effectiveUploadBudget = std::clamp<size_t>(
        std::max<size_t>(
            m_config.minProtectedUploadsPerFrame,
            static_cast<size_t>(std::llround(static_cast<double>(m_config.maxUploadsPerFrame) * upMul))),
        m_config.minProtectedUploadsPerFrame,
        m_config.maxUploadsPerFrame);
}

std::vector<WorldTileSystem::VisibleTileCandidate> WorldTileSystem::ComputeVisibleTiles(const ViewState& view) const {
    std::vector<VisibleTileCandidate> out;
    if (!m_initialized || !m_enabled) {
        return out;
    }

    struct ZoomBand {
        int zoom = 0;
        double radiusKm = 0.0;
        bool nearProtected = false;
        float basePriority = 0.0f;
        double centerLatDeg = 0.0;
        double centerLonDeg = 0.0;
    };

    const int bandIndex = std::clamp(m_activeZoomBand, 0, 3);
    const auto& radii = kBandRadiusKm[static_cast<size_t>(bandIndex)];
    const double lat = ClampLatDeg(view.latDeg);
    const double lon = WrapLonDeg(view.lonDeg);
    const double speedMps = std::clamp(std::abs(view.speedMps), 0.0, 450.0);
    const double headingDeg = view.headingDeg;

    std::array<ZoomBand, 3> bands{};
    for (size_t i = 0; i < bands.size(); ++i) {
        bands[i].zoom = std::clamp(m_activeZooms[i], m_config.minZoom, m_config.maxZoom);
        bands[i].radiusKm = radii[i];
        bands[i].nearProtected = (i == 0);
        bands[i].basePriority = static_cast<float>(i);
        bands[i].centerLatDeg = lat;
        bands[i].centerLonDeg = lon;

        const double radiusMeters = bands[i].radiusKm * 1000.0;
        const double lookaheadMeters = std::clamp(
            speedMps * kBandLookaheadSeconds[i] + radiusMeters * kBandLookaheadFractions[i],
            0.0,
            radiusMeters * kBandLookaheadCapFractions[i]);
        if (lookaheadMeters > 1.0) {
            OffsetLatLonByHeading(lat, lon, headingDeg, lookaheadMeters, bands[i].centerLatDeg, bands[i].centerLonDeg);
        }
    }

    struct CandidateState {
        uint8_t bandIndex = 0;
        bool nearProtected = false;
        bool highPriority = false;
        float priority = std::numeric_limits<float>::max();
    };
    std::unordered_map<WorldTileKey, CandidateState, WorldTileKeyHasher> unique;

    for (size_t bandIdx = 0; bandIdx < bands.size(); ++bandIdx) {
        const auto& band = bands[bandIdx];
        const int z = band.zoom;
        const int n = 1 << z;
        const double centerX = LonToTileXUnwrapped(band.centerLonDeg, z);
        const double centerY = LatToTileY(band.centerLatDeg, z);
        const int centerXi = static_cast<int>(std::floor(centerX));
        const int centerYi = static_cast<int>(std::floor(centerY));
        const double kmPerTile = std::max(0.02, ApproxKmPerTile(band.centerLatDeg, z));
        const int radiusTiles = std::clamp(static_cast<int>(std::ceil(band.radiusKm / kmPerTile)) + 1, 2, 80);

        for (int y = centerYi - radiusTiles; y <= centerYi + radiusTiles; ++y) {
            const int yClamp = std::clamp(y, 0, n - 1);
            for (int x = centerXi - radiusTiles; x <= centerXi + radiusTiles; ++x) {
                const double dx = static_cast<double>(x) - centerX;
                const double dy = static_cast<double>(y) - centerY;
                const double distSq = dx * dx + dy * dy;
                if (distSq > static_cast<double>(radiusTiles * radiusTiles) * 1.08) {
                    continue;
                }

                WorldTileKey key{};
                key.z = z;
                key.x = WrapTileX(x, n);
                key.y = yClamp;

                const float distNorm = static_cast<float>(std::sqrt(distSq) / std::max(1, radiusTiles));
                const double forwardTiles = HeadingForwardComponentTiles(dx, dy, headingDeg);
                const float forwardNorm = static_cast<float>(forwardTiles / std::max(1, radiusTiles));
                const float forwardBias = std::max(0.0f, forwardNorm);
                const float priority = band.basePriority + distNorm - forwardBias * (band.nearProtected ? 0.34f : 0.22f);
                auto& s = unique[key];
                if (priority < s.priority) {
                    s.priority = priority;
                    s.bandIndex = static_cast<uint8_t>(std::min<size_t>(bandIdx, 2));
                }
                const bool nearProtect =
                    band.nearProtected &&
                    (distNorm <= kNearProtectionDistNorm || (distNorm <= 0.72f && forwardNorm >= 0.18f));
                s.nearProtected = s.nearProtected || nearProtect;
                s.highPriority = s.highPriority ||
                    (band.nearProtected && (distNorm < 0.55f || (distNorm < 0.92f && forwardNorm >= 0.24f)));
            }
        }
    }

    out.reserve(unique.size());
    for (const auto& [key, state] : unique) {
        VisibleTileCandidate c{};
        c.key = key;
        c.bandIndex = state.bandIndex;
        c.nearProtected = state.nearProtected;
        c.highPriority = state.highPriority;
        c.priority = state.priority;
        out.push_back(c);
    }
    std::sort(out.begin(), out.end(), [](const VisibleTileCandidate& a, const VisibleTileCandidate& b) {
        if (a.priority == b.priority) {
            if (a.key.z == b.key.z) {
                if (a.key.y == b.key.y) {
                    return a.key.x < b.key.x;
                }
                return a.key.y < b.key.y;
            }
            return a.key.z > b.key.z;
        }
        return a.priority < b.priority;
    });
    if (out.size() > m_config.maxVisibleTilesPerFrame) {
        std::array<std::vector<VisibleTileCandidate>, 3> byBand{};
        for (const VisibleTileCandidate& candidate : out) {
            const size_t band = std::min<size_t>(candidate.bandIndex, byBand.size() - 1u);
            byBand[band].push_back(candidate);
        }

        std::array<float, 3> bandShare{0.45f, 0.33f, 0.22f};
        switch (m_activeZoomBand) {
            case 0:
                bandShare = {0.52f, 0.30f, 0.18f};
                break;
            case 1:
                bandShare = {0.46f, 0.32f, 0.22f};
                break;
            case 2:
                bandShare = {0.38f, 0.33f, 0.29f};
                break;
            default:
                bandShare = {0.30f, 0.33f, 0.37f};
                break;
        }

        const size_t cap = m_config.maxVisibleTilesPerFrame;
        std::array<size_t, 3> takeCount{};
        size_t taken = 0;
        for (size_t i = 0; i < byBand.size(); ++i) {
            takeCount[i] = std::min<size_t>(
                byBand[i].size(),
                static_cast<size_t>(std::floor(static_cast<double>(cap) * static_cast<double>(bandShare[i]))));
            taken += takeCount[i];
        }

        std::vector<VisibleTileCandidate> limited;
        limited.reserve(cap);
        std::array<size_t, 3> nextIndex{};
        for (size_t i = 0; i < byBand.size(); ++i) {
            for (size_t j = 0; j < takeCount[i]; ++j) {
                limited.push_back(byBand[i][j]);
            }
            nextIndex[i] = takeCount[i];
        }

        while (limited.size() < cap) {
            size_t bestBand = byBand.size();
            float bestPriority = std::numeric_limits<float>::max();
            for (size_t i = 0; i < byBand.size(); ++i) {
                if (nextIndex[i] >= byBand[i].size()) {
                    continue;
                }
                const float priority = byBand[i][nextIndex[i]].priority;
                if (bestBand == byBand.size() || priority < bestPriority) {
                    bestBand = i;
                    bestPriority = priority;
                }
            }
            if (bestBand == byBand.size()) {
                break;
            }
            limited.push_back(byBand[bestBand][nextIndex[bestBand]]);
            nextIndex[bestBand] += 1;
        }
        out.swap(limited);
    }
    return out;
}

void WorldTileSystem::TouchResident(const WorldTileKey& key, TileState& state) {
    state.lastTouchedFrame = m_frameId;
    if (!state.inLru) {
        m_residentLru.push_front(key);
        state.lruIt = m_residentLru.begin();
        state.inLru = true;
    } else {
        m_residentLru.splice(m_residentLru.begin(), m_residentLru, state.lruIt);
        state.lruIt = m_residentLru.begin();
    }
}

void WorldTileSystem::ReleasePageTableSlot(const WorldTileKey& key) {
    const auto slotIt = m_keyToPageTableSlot.find(key);
    if (slotIt == m_keyToPageTableSlot.end()) {
        return;
    }
    const uint32_t slot = slotIt->second;
    if (slot < m_pageTableSlots.size()) {
        m_pageTableSlots[slot].reset();
        if (slot < m_pageTableEverUsed.size() && m_pageTableEverUsed[slot] != 0u) {
            m_pageTableTombstoneCount += 1;
        }
        const uint32_t x = slot % static_cast<uint32_t>(m_config.pageTableWidth);
        const uint32_t y = slot / static_cast<uint32_t>(m_config.pageTableWidth);
        // Mark deleted entries as tombstones so probe chains remain valid.
        m_pendingPageUpdates.push_back(WorldPageTableUpdate{x, y, 0u, 0u, kPageTableTombstoneValue});
        m_pageTableWrites += 1;
    }
    m_keyToPageTableSlot.erase(slotIt);
}

std::optional<uint32_t> WorldTileSystem::AssignPageTableSlot(const WorldTileKey& key) {
    const auto existing = m_keyToPageTableSlot.find(key);
    if (existing != m_keyToPageTableSlot.end()) {
        return existing->second;
    }
    if (m_pageTableSlots.empty()) {
        return std::nullopt;
    }

    const size_t count = m_pageTableSlots.size();
    const size_t maxProbeCount = std::min<size_t>(count, std::max<size_t>(1u, m_config.shaderProbeBudget));
    size_t slot = static_cast<size_t>(HashTileKeyForPageTable(key)) % count;
    std::optional<size_t> firstTombstone;
    for (size_t i = 0; i < maxProbeCount; ++i) {
        const size_t probe = (slot + i) % count;
        if (!m_pageTableSlots[probe].has_value()) {
            if (probe < m_pageTableEverUsed.size() && m_pageTableEverUsed[probe] != 0u) {
                if (!firstTombstone.has_value()) {
                    firstTombstone = probe;
                }
                continue;
            }
            const size_t target = firstTombstone.value_or(probe);
            m_pageTableSlots[target] = key;
            if (firstTombstone.has_value() && target == *firstTombstone && m_pageTableTombstoneCount > 0) {
                m_pageTableTombstoneCount -= 1;
            }
            if (target < m_pageTableEverUsed.size()) {
                m_pageTableEverUsed[target] = 1u;
            }
            m_keyToPageTableSlot.emplace(key, static_cast<uint32_t>(target));
            return static_cast<uint32_t>(target);
        }
    }
    if (firstTombstone.has_value()) {
        const size_t target = *firstTombstone;
        m_pageTableSlots[target] = key;
        if (m_pageTableTombstoneCount > 0) {
            m_pageTableTombstoneCount -= 1;
        }
        if (target < m_pageTableEverUsed.size()) {
            m_pageTableEverUsed[target] = 1u;
        }
        m_keyToPageTableSlot.emplace(key, static_cast<uint32_t>(target));
        return static_cast<uint32_t>(target);
    }
    return std::nullopt;
}

uint32_t WorldTileSystem::PackPageTableKey0(const WorldTileKey& key) {
    const uint32_t zBits = static_cast<uint32_t>(std::clamp(key.z, 0, 63));
    const uint32_t xBits = static_cast<uint32_t>(std::clamp(key.x, 0, 0x03FFFFFF));
    return (zBits << 26u) | xBits;
}

uint32_t WorldTileSystem::PackPageTableKey1(const WorldTileKey& key) {
    return static_cast<uint32_t>(std::clamp(key.y, 0, 0x03FFFFFF));
}

uint32_t WorldTileSystem::PackPageTableValue(const WorldTileKey& key, int atlasPageIndex) {
    (void)key;
    const uint32_t pageBits = static_cast<uint32_t>(std::clamp(atlasPageIndex, 0, 0x00FFFFFF));
    return 0x80000000u | pageBits;
}

void WorldTileSystem::EvictResident(const WorldTileKey& key, TileState& state, bool returnPageToFreeList) {
    if (!state.resident || state.atlasPageIndex < 0) {
        return;
    }
    if (state.inLru) {
        m_residentLru.erase(state.lruIt);
        state.inLru = false;
    }

    const int page = state.atlasPageIndex;
    if (page >= 0 && static_cast<size_t>(page) < m_pageToKey.size()) {
        m_pageToKey[static_cast<size_t>(page)].reset();
        if (returnPageToFreeList) {
            m_freeAtlasPages.push_back(page);
        }
    }
    state.resident = false;
    state.atlasPageIndex = -1;
    state.pageTableProbeDistance = 0;
    ReleasePageTableSlot(key);
    m_evictions += 1;
}

std::optional<int> WorldTileSystem::AllocateAtlasPage(const WorldTileKey& forKey, bool protectedRequest) {
    if (!m_freeAtlasPages.empty()) {
        if (!protectedRequest) {
            size_t protectedResidentCount = 0;
            for (const auto& [_, state] : m_tiles) {
                if (state.resident && state.protectedNear) {
                    protectedResidentCount += 1;
                }
            }
            const size_t reserveShortfall =
                (protectedResidentCount >= m_config.nearResidentReservePages)
                ? 0u
                : (m_config.nearResidentReservePages - protectedResidentCount);
            if (m_freeAtlasPages.size() <= reserveShortfall) {
                return std::nullopt;
            }
        }
        const int page = m_freeAtlasPages.back();
        m_freeAtlasPages.pop_back();
        return page;
    }

    for (auto it = m_residentLru.rbegin(); it != m_residentLru.rend(); ++it) {
        const WorldTileKey evictKey = *it;
        auto found = m_tiles.find(evictKey);
        if (found == m_tiles.end()) {
            continue;
        }
        TileState& state = found->second;
        if (!state.resident || state.protectedNear || state.visibleThisFrame || evictKey == forKey) {
            continue;
        }
        const bool stickyHighDetail = evictKey.z >= std::max(m_activeZooms[1] - 1, m_config.minZoom);
        if (stickyHighDetail && m_config.residentStickinessFrames > 0 &&
            (m_frameId - state.lastTouchedFrame) <= m_config.residentStickinessFrames) {
            m_evictionSkipsSticky += 1;
            continue;
        }
        const int page = state.atlasPageIndex;
        EvictResident(evictKey, state, false);
        if (page >= 0) {
            return page;
        }
    }
    return std::nullopt;
}

void WorldTileSystem::GarbageCollect() {
    if (m_tiles.empty()) {
        return;
    }
    std::vector<WorldTileKey> stale;
    stale.reserve(m_tiles.size() / 4);
    for (const auto& [key, state] : m_tiles) {
        if (state.resident || state.pendingUpload || state.visibleThisFrame) {
            continue;
        }
        if (m_frameId > state.lastTouchedFrame && (m_frameId - state.lastTouchedFrame) > m_config.staleNonResidentFrames) {
            stale.push_back(key);
        }
    }
    for (const WorldTileKey& key : stale) {
        auto found = m_tiles.find(key);
        if (found == m_tiles.end()) {
            continue;
        }
        TileState& state = found->second;
        if (state.resident) {
            EvictResident(key, state);
        } else {
            ReleasePageTableSlot(key);
            if (state.inLru) {
                m_residentLru.erase(state.lruIt);
                state.inLru = false;
            }
            m_tiles.erase(found);
        }
    }
}

void WorldTileSystem::Tick(const ViewState& view, SatelliteStreamer& streamer) {
    if (!m_initialized || !m_enabled) {
        return;
    }

    m_frameId += 1;
    m_pendingAtlasUploads.clear();
    m_pendingPageUpdates.clear();
    m_debugMaxProbeDistance = 0;
    m_debugAvgProbeDistance = 0.0;
    m_debugVisibleResidentCount = 0;
    m_debugVisibleResidentOverProbeBudget = 0;
    uint64_t probeAccum = 0;

    UpdateZoomBandWithHysteresis(view.altitudeMeters);
    UpdateFrameTimeGovernor();

    for (auto& [_, state] : m_tiles) {
        state.visibleThisFrame = false;
        state.protectedNear = false;
    }

    const auto visible = ComputeVisibleTiles(view);
    m_visibleTileCount = visible.size();

    size_t requestBudget = m_effectiveRequestBudget;
    size_t uploadBudget = m_effectiveUploadBudget;
    size_t protectedRequestBudget = std::min(requestBudget, m_config.minProtectedRequestsPerFrame);
    size_t protectedUploadBudget = std::min(uploadBudget, m_config.minProtectedUploadsPerFrame);
    for (const VisibleTileCandidate& candidate : visible) {
        TileState& state = m_tiles[candidate.key];
        state.visibleThisFrame = true;
        state.protectedNear = state.protectedNear || candidate.nearProtected;
        state.lastTouchedFrame = m_frameId;

        if (state.resident) {
            m_debugVisibleResidentCount += 1;
            m_debugMaxProbeDistance = std::max(m_debugMaxProbeDistance, state.pageTableProbeDistance);
            probeAccum += static_cast<uint64_t>(state.pageTableProbeDistance);
            if (state.pageTableProbeDistance >= m_config.shaderProbeBudget) {
                m_debugVisibleResidentOverProbeBudget += 1;
            }
            TouchResident(candidate.key, state);
            continue;
        }

        if (!state.pendingUpload) {
            std::shared_ptr<const std::vector<uint8_t>> pixels;
            if (streamer.TryGetCachedTileRgba(candidate.key.z, candidate.key.x, candidate.key.y, pixels) &&
                HasDecodedTilePixels(pixels)) {
                state.decodedPixels = std::move(pixels);
                state.pendingUpload = true;
                m_streamerCacheHits += 1;
            } else {
                m_streamerCacheMisses += 1;
            }
        }
    }

    auto tryQueueRequests = [&](bool protectedOnly, size_t& phaseBudget) {
        for (const VisibleTileCandidate& candidate : visible) {
            if (phaseBudget == 0 || requestBudget == 0) {
                break;
            }
            if (protectedOnly && !candidate.nearProtected) {
                continue;
            }
            auto found = m_tiles.find(candidate.key);
            if (found == m_tiles.end()) {
                continue;
            }
            TileState& state = found->second;
            if (state.resident || state.pendingUpload) {
                continue;
            }
            const bool allowRequest = (m_frameId - state.lastRequestFrame) >= m_config.requestCooldownFrames;
            if (!allowRequest) {
                continue;
            }
            if (streamer.QueueTileRequest(candidate.key.z, candidate.key.x, candidate.key.y, candidate.highPriority)) {
                state.lastRequestFrame = m_frameId;
                phaseBudget -= 1;
                requestBudget -= 1;
                m_streamerRequests += 1;
            }
        }
    };

    tryQueueRequests(true, protectedRequestBudget);
    if (requestBudget > 0) {
        size_t remainingRequestBudget = requestBudget;
        tryQueueRequests(false, remainingRequestBudget);
        requestBudget = remainingRequestBudget;
    }

    auto tryUploadVisible = [&](bool protectedOnly, size_t& phaseBudget) {
        for (const VisibleTileCandidate& candidate : visible) {
            if (phaseBudget == 0 || uploadBudget == 0) {
                break;
            }
            if (protectedOnly && !candidate.nearProtected) {
                continue;
            }
            auto found = m_tiles.find(candidate.key);
            if (found == m_tiles.end()) {
                continue;
            }
            TileState& state = found->second;
            if (!state.pendingUpload || !state.decodedPixels || state.decodedPixels->size() < (256u * 256u * 4u)) {
                continue;
            }

            const auto pageOpt = AllocateAtlasPage(candidate.key, candidate.nearProtected);
            if (!pageOpt.has_value()) {
                m_uploadSkipsNoPage += 1;
                continue;
            }

            const auto slotOpt = AssignPageTableSlot(candidate.key);
            if (!slotOpt.has_value()) {
                m_freeAtlasPages.push_back(*pageOpt);
                m_uploadSkipsNoPage += 1;
                continue;
            }

            const int page = *pageOpt;
            const uint32_t slot = *slotOpt;
            state.resident = true;
            state.pendingUpload = false;
            state.atlasPageIndex = page;
            m_pageToKey[static_cast<size_t>(page)] = candidate.key;
            TouchResident(candidate.key, state);

            WorldAtlasUpload upload{};
            upload.key = candidate.key;
            upload.atlasPageX = static_cast<uint32_t>(page % m_config.atlasPagesX);
            upload.atlasPageY = static_cast<uint32_t>(page / m_config.atlasPagesX);
            upload.rgbaPixels = state.decodedPixels;
            PopulateUploadNeighborPixels(candidate.key, streamer, upload);
            m_pendingAtlasUploads.push_back(std::move(upload));
            m_atlasUploads += 1;

            const size_t tableCount = static_cast<size_t>(m_config.pageTableWidth) * static_cast<size_t>(m_config.pageTableHeight);
            const size_t hashSlot = (tableCount > 0)
                ? (static_cast<size_t>(HashTileKeyForPageTable(candidate.key)) % tableCount)
                : 0u;
            const size_t slotSz = static_cast<size_t>(slot);
            const size_t probeDistance = (slotSz >= hashSlot) ? (slotSz - hashSlot) : (slotSz + tableCount - hashSlot);
            state.pageTableProbeDistance = static_cast<uint32_t>(std::min<size_t>(probeDistance, 0xFFFFFFFFu));
            const uint32_t x = slot % static_cast<uint32_t>(m_config.pageTableWidth);
            const uint32_t y = slot / static_cast<uint32_t>(m_config.pageTableWidth);
            const uint32_t value = PackPageTableValue(candidate.key, page);
            const uint32_t key0 = PackPageTableKey0(candidate.key);
            const uint32_t key1 = PackPageTableKey1(candidate.key);
            m_pendingPageUpdates.push_back(WorldPageTableUpdate{x, y, key0, key1, value});
            m_pageTableWrites += 1;

            phaseBudget -= 1;
            uploadBudget -= 1;
        }
    };

    tryUploadVisible(true, protectedUploadBudget);
    if (uploadBudget > 0) {
        size_t remainingUploadBudget = uploadBudget;
        tryUploadVisible(false, remainingUploadBudget);
        uploadBudget = remainingUploadBudget;
    }

    if (m_debugVisibleResidentCount > 0) {
        m_debugAvgProbeDistance =
            static_cast<double>(probeAccum) / static_cast<double>(m_debugVisibleResidentCount);
    }

    GarbageCollect();
}

void WorldTileSystem::ConsumeGpuUpdates(std::vector<WorldAtlasUpload>& outAtlas, std::vector<WorldPageTableUpdate>& outPageTable) {
    outAtlas.clear();
    outPageTable.clear();
    outAtlas.swap(m_pendingAtlasUploads);
    outPageTable.swap(m_pendingPageUpdates);
}

WorldTileSystemStats WorldTileSystem::GetStats() const {
    WorldTileSystemStats s{};
    s.enabled = m_enabled && m_initialized;
    s.frameId = m_frameId;
    s.visibleTileCount = m_visibleTileCount;
    s.pageTableOccupancy = m_keyToPageTableSlot.size();
    s.streamerRequests = m_streamerRequests;
    s.streamerCacheHits = m_streamerCacheHits;
    s.streamerCacheMisses = m_streamerCacheMisses;
    s.atlasUploads = m_atlasUploads;
    s.pageTableWrites = m_pageTableWrites;
    s.evictions = m_evictions;
    s.uploadSkipsNoPage = m_uploadSkipsNoPage;
    s.evictionSkipsSticky = m_evictionSkipsSticky;
    s.activeZoomNear = m_activeZooms[0];
    s.activeZoomMid = m_activeZooms[1];
    s.activeZoomFar = m_activeZooms[2];
    s.activeZoomBand = m_activeZoomBand;
    const uint64_t holdFrames = m_config.zoomSwitchHoldFrames;
    const uint64_t elapsedSinceSwitch = m_frameId - m_lastZoomBandSwitchFrame;
    s.zoomBandHoldFramesRemaining = (elapsedSinceSwitch >= holdFrames) ? 0 : (holdFrames - elapsedSinceSwitch);
    s.governorLevel = m_governorLevel;
    s.governorRequestBudget = m_effectiveRequestBudget;
    s.governorUploadBudget = m_effectiveUploadBudget;
    s.governorFrameTimeMs = m_lastFrameTimeMs;
    s.pageTableCapacity = m_pageTableSlots.size();
    s.pageTableTombstoneCount = m_pageTableTombstoneCount;
    s.pageTableLoadFactor = (s.pageTableCapacity > 0)
        ? static_cast<double>(s.pageTableOccupancy) / static_cast<double>(s.pageTableCapacity)
        : 0.0;
    s.pageTableTombstoneRatio = (s.pageTableCapacity > 0)
        ? static_cast<double>(s.pageTableTombstoneCount) / static_cast<double>(s.pageTableCapacity)
        : 0.0;
    s.shaderProbeBudget = m_config.shaderProbeBudget;
    s.maxProbeDistance = m_debugMaxProbeDistance;
    s.avgProbeDistance = m_debugAvgProbeDistance;
    s.visibleResidentCount = m_debugVisibleResidentCount;
    s.visibleResidentOverProbeBudget = m_debugVisibleResidentOverProbeBudget;

    size_t resident = 0;
    size_t protectedCount = 0;
    size_t pending = 0;
    for (const auto& [_, state] : m_tiles) {
        if (state.resident) {
            resident += 1;
        }
        if (state.protectedNear) {
            protectedCount += 1;
        }
        if (state.pendingUpload) {
            pending += 1;
        }
    }
    s.residentTileCount = resident;
    s.protectedTileCount = protectedCount;
    s.pendingUploadCount = pending;
    return s;
}

} // namespace flight::satellite
