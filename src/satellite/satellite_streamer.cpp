#include "satellite/satellite_streamer.h"

#include <windows.h>
#include <winhttp.h>
#include <wincodec.h>
#include <wrl/client.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "sim.h"

namespace flight::satellite {
namespace {

using Microsoft::WRL::ComPtr;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusMeters = flight::kEarthRadiusMeters;

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

int WrapTileX(int x, int n) {
    int r = x % n;
    if (r < 0) {
        r += n;
    }
    return r;
}

double LonToTileXUnwrapped(double lonDeg, int z) {
    const double n = static_cast<double>(1u << z);
    return (lonDeg + 180.0) / 360.0 * n;
}

double LatToTileY(double latDeg, int z) {
    const double n = static_cast<double>(1u << z);
    const double latRad = flight::DegToRad(ClampLatDeg(latDeg));
    const double y = (1.0 - std::log(std::tan(latRad) + 1.0 / std::cos(latRad)) / kPi) * 0.5 * n;
    return std::clamp(y, 0.0, n - 1.0e-6);
}

double LatDegToMercatorY(double latDeg) {
    const double latRad = flight::DegToRad(ClampLatDeg(latDeg));
    return std::log(std::tan(latRad) + 1.0 / std::cos(latRad));
}

double MercatorYToLatDeg(double mercatorY) {
    return flight::RadToDeg(std::atan(std::sinh(mercatorY)));
}

double TileYToLatDeg(double y, int z) {
    const double n = static_cast<double>(1u << z);
    const double merc = kPi * (1.0 - 2.0 * (y / n));
    return flight::RadToDeg(std::atan(std::sinh(merc)));
}

double DistanceMeters(double latA, double lonA, double latB, double lonB) {
    const double latARad = flight::DegToRad(latA);
    const double lonARad = flight::DegToRad(lonA);
    const double latBRad = flight::DegToRad(latB);
    const double lonBRad = flight::DegToRad(lonB);
    const double dLat = latBRad - latARad;
    const double dLon = lonBRad - lonARad;
    const double s0 = std::sin(dLat * 0.5);
    const double s1 = std::sin(dLon * 0.5);
    const double a = s0 * s0 + std::cos(latARad) * std::cos(latBRad) * s1 * s1;
    const double c = 2.0 * std::asin(std::sqrt(std::clamp(a, 0.0, 1.0)));
    return kEarthRadiusMeters * c;
}

double HeadingForwardComponentTiles(double dx, double dy, double headingDeg) {
    const double headingRad = flight::DegToRad(headingDeg);
    const double east = std::sin(headingRad);
    const double north = std::cos(headingRad);
    return dx * east + (-dy) * north;
}

double SnapToStep(double value, double step) {
    if (step <= 1e-9) {
        return value;
    }
    return std::round(value / step) * step;
}

std::wstring Utf8ToWide(const std::string& in) {
    if (in.empty()) {
        return {};
    }
    const int count = MultiByteToWideChar(CP_UTF8, 0, in.c_str(), static_cast<int>(in.size()), nullptr, 0);
    if (count <= 0) {
        return {};
    }
    std::wstring out(static_cast<size_t>(count), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, in.c_str(), static_cast<int>(in.size()), out.data(), count);
    return out;
}

std::string ReplaceAll(std::string value, const std::string& needle, const std::string& repl) {
    size_t pos = 0;
    while ((pos = value.find(needle, pos)) != std::string::npos) {
        value.replace(pos, needle.size(), repl);
        pos += repl.size();
    }
    return value;
}

} // namespace

struct SatelliteStreamer::Impl {
    struct TileKey {
        int z = 0;
        int x = 0;
        int y = 0;
        friend bool operator==(const TileKey& a, const TileKey& b) {
            return a.z == b.z && a.x == b.x && a.y == b.y;
        }
    };

    struct TileKeyHasher {
        size_t operator()(const TileKey& key) const noexcept {
            const uint64_t a = static_cast<uint64_t>(static_cast<uint32_t>(key.z));
            const uint64_t b = static_cast<uint64_t>(static_cast<uint32_t>(key.x));
            const uint64_t c = static_cast<uint64_t>(static_cast<uint32_t>(key.y));
            return static_cast<size_t>((a << 48u) ^ (b << 24u) ^ c);
        }
    };

    struct CachedTile {
        std::shared_ptr<std::vector<uint8_t>> rgbaPixels;
        std::list<TileKey>::iterator lruIt;
    };

    struct RingParams {
        double radiusKm = 0.0;
        int zoom = 0;
        int textureSize = 1024;
        int prefetchRadiusTiles = 4;
    };

    Config config{};
    bool initialized = false;
    bool stopWorkers = false;
    std::vector<std::thread> workers;
    mutable std::mutex mutex;
    std::condition_variable cv;
    std::deque<TileKey> requestQueue;
    std::unordered_set<TileKey, TileKeyHasher> queuedSet;

    std::unordered_map<TileKey, CachedTile, TileKeyHasher> tileCache;
    std::list<TileKey> lru;
    std::deque<TileKey> dirtyTileQueue;
    std::unordered_set<TileKey, TileKeyHasher> dirtyTileSet;

    std::atomic<uint64_t> diskHits{0};
    std::atomic<uint64_t> networkHits{0};
    std::atomic<uint64_t> networkFailures{0};
    std::atomic<uint64_t> decodeFailures{0};

    std::atomic<bool> dirtySinceLastCompose{true};
    std::array<int, 3> activeZooms{13, 11, 9};
    double lastComposeLatDeg = 1000.0;
    double lastComposeLonDeg = 1000.0;
    double lastNearComposeLatDeg = 1000.0;
    double lastNearComposeLonDeg = 1000.0;
    double lastMidFarComposeLatDeg = 1000.0;
    double lastMidFarComposeLonDeg = 1000.0;
    std::array<int, 3> lastComposeZooms{-1, -1, -1};
    std::array<double, 3> lastComposeRadiusKm{0.0, 0.0, 0.0};
    std::array<RingTexture, 3> lastComposedRings{};
    std::array<bool, 3> hasComposedRing{false, false, false};
    bool hasComposeState = false;

    HINTERNET hSession = nullptr;

    ~Impl() {
        Shutdown();
    }

    bool Initialize(const Config& cfg, std::string& error) {
        Shutdown();
        config = cfg;
        config.workerCount = std::clamp(config.workerCount, 1, 16);
        config.maxMemoryTiles = std::max<size_t>(config.maxMemoryTiles, 128);
        config.nearTextureSize = std::clamp(config.nearTextureSize, 256, 2048);
        config.midTextureSize = std::clamp(config.midTextureSize, 256, 2048);
        config.farTextureSize = std::clamp(config.farTextureSize, 256, 2048);

        std::error_code ec;
        std::filesystem::create_directories(config.diskCacheDirectory, ec);
        if (ec) {
            error = "Failed creating satellite cache directory: " + ec.message();
            return false;
        }

        hSession = WinHttpOpen(
            L"3DFlight/1.0",
            WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY,
            WINHTTP_NO_PROXY_NAME,
            WINHTTP_NO_PROXY_BYPASS,
            0);
        if (!hSession) {
            error = "WinHttpOpen failed";
            return false;
        }
        DWORD maxConnections = static_cast<DWORD>(std::clamp(config.workerCount * 2, 8, 32));
        (void)WinHttpSetOption(hSession, WINHTTP_OPTION_MAX_CONNS_PER_SERVER, &maxConnections, sizeof(maxConnections));
        (void)WinHttpSetOption(hSession, WINHTTP_OPTION_MAX_CONNS_PER_1_0_SERVER, &maxConnections, sizeof(maxConnections));

        stopWorkers = false;
        for (int i = 0; i < config.workerCount; ++i) {
            workers.emplace_back([this]() { WorkerLoop(); });
        }
        initialized = true;
        return true;
    }

    void Shutdown() {
        {
            std::scoped_lock<std::mutex> lock(mutex);
            stopWorkers = true;
            requestQueue.clear();
            queuedSet.clear();
        }
        cv.notify_all();
        for (auto& w : workers) {
            if (w.joinable()) {
                w.join();
            }
        }
        workers.clear();

        if (hSession) {
            WinHttpCloseHandle(hSession);
            hSession = nullptr;
        }

        {
            std::scoped_lock<std::mutex> lock(mutex);
            tileCache.clear();
            lru.clear();
            dirtyTileQueue.clear();
            dirtyTileSet.clear();
        }
        hasComposeState = false;
        hasComposedRing = {false, false, false};
        lastComposeZooms = {-1, -1, -1};
        lastComposeRadiusKm = {0.0, 0.0, 0.0};
        lastComposeLatDeg = 1000.0;
        lastComposeLonDeg = 1000.0;
        lastNearComposeLatDeg = 1000.0;
        lastNearComposeLonDeg = 1000.0;
        lastMidFarComposeLatDeg = 1000.0;
        lastMidFarComposeLonDeg = 1000.0;
        dirtySinceLastCompose = true;
        initialized = false;
    }

    std::array<RingParams, 3> ComputeRingParams(double altitudeMeters) {
        std::array<RingParams, 3> rings{};
        const double altitudeKm = std::max(0.0, altitudeMeters * 0.001);
        const double horizonKm =
            std::sqrt(std::max(0.0, (kEarthRadiusMeters + altitudeMeters) * (kEarthRadiusMeters + altitudeMeters) -
                                         (kEarthRadiusMeters * kEarthRadiusMeters))) *
            0.001;

        // Scale stream coverage with altitude so cruising flight keeps streamed imagery much farther out.
        const double farRadiusKm = std::clamp(horizonKm * 1.45 + 100.0, 220.0, 1400.0);
        const double midRadiusKm = std::clamp(farRadiusKm * 0.48, 45.0, 520.0);
        const double nearRadiusKm = std::clamp(midRadiusKm * 0.30, 10.0, 150.0);

        rings[0].radiusKm = nearRadiusKm;
        rings[1].radiusKm = midRadiusKm;
        rings[2].radiusKm = farRadiusKm;
        rings[0].textureSize = config.nearTextureSize;
        rings[1].textureSize = config.midTextureSize;
        rings[2].textureSize = config.farTextureSize;

        if (altitudeKm < 0.5) {
            rings[0].zoom = 16;
            rings[1].zoom = 14;
            rings[2].zoom = 12;
        } else if (altitudeKm < 2.0) {
            rings[0].zoom = 15;
            rings[1].zoom = 13;
            rings[2].zoom = 11;
        } else if (altitudeKm < 6.0) {
            rings[0].zoom = 14;
            rings[1].zoom = 12;
            rings[2].zoom = 10;
        } else {
            rings[0].zoom = 13;
            rings[1].zoom = 11;
            rings[2].zoom = 9;
        }

        auto prefetchTilesForRadius = [](double radiusKm, int zoom) {
            const double tilesAtZoom = static_cast<double>(1u << zoom);
            const double kmPerTileAtEquator = 40075.016686 / tilesAtZoom;
            const int needed = static_cast<int>(std::ceil(radiusKm / std::max(kmPerTileAtEquator, 1e-3))) + 2;
            return std::clamp(needed, 4, 26);
        };
        rings[0].prefetchRadiusTiles = prefetchTilesForRadius(rings[0].radiusKm, rings[0].zoom);
        rings[1].prefetchRadiusTiles = prefetchTilesForRadius(rings[1].radiusKm, rings[1].zoom);
        rings[2].prefetchRadiusTiles = prefetchTilesForRadius(rings[2].radiusKm, rings[2].zoom);

        activeZooms = {rings[0].zoom, rings[1].zoom, rings[2].zoom};
        return rings;
    }

    bool EnqueueTileRequest(const TileKey& key, bool highPriority) {
        std::scoped_lock<std::mutex> lock(mutex);
        if (tileCache.find(key) != tileCache.end()) {
            TouchLru(key);
            return false;
        }
        if (queuedSet.find(key) != queuedSet.end()) {
            return false;
        }
        if (highPriority) {
            requestQueue.push_front(key);
        } else {
            requestQueue.push_back(key);
        }
        queuedSet.insert(key);
        cv.notify_one();
        return true;
    }

    size_t QueueBounds(
        int zoom,
        double latDeg,
        double lonDeg,
        double headingDeg,
        double radiusKm,
        int capRadiusTiles,
        bool highPriority,
        size_t maxEnqueue = std::numeric_limits<size_t>::max()) {
        const double latRad = flight::DegToRad(latDeg);
        const double radiusMeters = radiusKm * 1000.0;
        const double dLatDeg = flight::RadToDeg(radiusMeters / kEarthRadiusMeters);
        const double cosLat = std::max(0.15, std::cos(latRad));
        const double dLonDeg = flight::RadToDeg(radiusMeters / (kEarthRadiusMeters * cosLat));

        const double lonW = lonDeg - dLonDeg;
        const double lonE = lonDeg + dLonDeg;
        const double latS = std::clamp(latDeg - dLatDeg, -85.0, 85.0);
        const double latN = std::clamp(latDeg + dLatDeg, -85.0, 85.0);

        const int n = 1 << zoom;
        const double centerX = LonToTileXUnwrapped(lonDeg, zoom);
        const double centerY = LatToTileY(latDeg, zoom);

        const int capX0 = static_cast<int>(std::floor(centerX)) - capRadiusTiles;
        const int capX1 = static_cast<int>(std::floor(centerX)) + capRadiusTiles;
        const int capY0 = std::max(0, static_cast<int>(std::floor(centerY)) - capRadiusTiles);
        const int capY1 = std::min(n - 1, static_cast<int>(std::floor(centerY)) + capRadiusTiles);

        int x0 = static_cast<int>(std::floor(LonToTileXUnwrapped(lonW, zoom)));
        int x1 = static_cast<int>(std::floor(LonToTileXUnwrapped(lonE, zoom)));
        if (x0 > x1) {
            std::swap(x0, x1);
        }
        x0 = std::max(x0, capX0);
        x1 = std::min(x1, capX1);

        int y0 = static_cast<int>(std::floor(LatToTileY(latN, zoom)));
        int y1 = static_cast<int>(std::floor(LatToTileY(latS, zoom)));
        if (y0 > y1) {
            std::swap(y0, y1);
        }
        y0 = std::max(y0, capY0);
        y1 = std::min(y1, capY1);

        struct Candidate {
            TileKey key{};
            double distSq = 0.0;
            double score = 0.0;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                Candidate c{};
                c.key.z = zoom;
                c.key.x = WrapTileX(x, n);
                c.key.y = std::clamp(y, 0, n - 1);
                const double dx = static_cast<double>(x) - centerX;
                const double dy = static_cast<double>(y) - centerY;
                c.distSq = dx * dx + dy * dy;
                const double forwardTiles = HeadingForwardComponentTiles(dx, dy, headingDeg);
                const double forwardNorm = forwardTiles / std::max(1, capRadiusTiles);
                const double forwardBias = std::max(0.0, forwardNorm);
                const double rearPenalty = std::max(0.0, -forwardNorm);
                c.score = c.distSq - forwardBias * (highPriority ? 3.4 : 2.2) + rearPenalty * (highPriority ? 0.45 : 0.25);
                candidates.push_back(c);
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            if (a.score == b.score) {
                if (a.distSq == b.distSq) {
                    if (a.key.y == b.key.y) {
                        return a.key.x < b.key.x;
                    }
                    return a.key.y < b.key.y;
                }
                return a.distSq < b.distSq;
            }
            return a.score < b.score;
        });

        size_t enqueued = 0;
        for (const Candidate& c : candidates) {
            if (enqueued >= maxEnqueue) {
                break;
            }
            if (EnqueueTileRequest(c.key, highPriority)) {
                enqueued += 1;
            }
        }
        return enqueued;
    }

    void PrefetchForView(double latDeg, double lonDeg, double altitudeMeters, double headingDeg) {
        if (!initialized) {
            return;
        }
        const auto rings = ComputeRingParams(altitudeMeters);
        size_t queued = 0;
        {
            std::scoped_lock<std::mutex> lock(mutex);
            queued = requestQueue.size();
        }

        if (queued > 12000) {
            return;
        }

        // Prioritize immediate surroundings first, then progressively expand.
        const int nearCoreRadius = std::clamp(rings[0].prefetchRadiusTiles / 3, 2, 8);
        const size_t nearCoreBudget = (queued < 1200) ? 480 : 192;
        size_t added = QueueBounds(
            rings[0].zoom,
            latDeg,
            lonDeg,
            headingDeg,
            rings[0].radiusKm,
            nearCoreRadius,
            true,
            nearCoreBudget);

        size_t queueNow = queued + added;
        const size_t nearExpandBudget = (queueNow < 1800) ? 720 : ((queueNow < 3200) ? 320 : 128);
        added += QueueBounds(
            rings[0].zoom,
            latDeg,
            lonDeg,
            headingDeg,
            rings[0].radiusKm,
            rings[0].prefetchRadiusTiles,
            false,
            nearExpandBudget);
        queueNow = queued + added;

        if (queueNow < 6200) {
            const size_t midBudget = (queueNow < 2600) ? 420 : 192;
            added += QueueBounds(
                rings[1].zoom,
                latDeg,
                lonDeg,
                headingDeg,
                rings[1].radiusKm,
                rings[1].prefetchRadiusTiles,
                false,
                midBudget);
            queueNow = queued + added;
        }

        if (queueNow < 7800) {
            const size_t farBudget = (queueNow < 3200) ? 260 : 128;
            (void)QueueBounds(
                rings[2].zoom,
                latDeg,
                lonDeg,
                headingDeg,
                rings[2].radiusKm,
                rings[2].prefetchRadiusTiles,
                false,
                farBudget);
        }
    }

    std::filesystem::path CachePathForTile(const TileKey& key) const {
        return config.diskCacheDirectory / std::to_string(key.z) / std::to_string(key.x) / (std::to_string(key.y) + ".jpg");
    }

    std::string BuildTileUrl(const TileKey& key) const {
        std::string url = config.urlTemplate;
        url = ReplaceAll(std::move(url), "{z}", std::to_string(key.z));
        url = ReplaceAll(std::move(url), "{x}", std::to_string(key.x));
        url = ReplaceAll(std::move(url), "{y}", std::to_string(key.y));
        return url;
    }

    bool ReadFileBytes(const std::filesystem::path& path, std::vector<uint8_t>& out) {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return false;
        }
        in.seekg(0, std::ios::end);
        const std::streamsize size = in.tellg();
        if (size <= 0) {
            return false;
        }
        in.seekg(0, std::ios::beg);
        out.resize(static_cast<size_t>(size));
        in.read(reinterpret_cast<char*>(out.data()), size);
        return static_cast<std::streamsize>(in.gcount()) == size;
    }

    bool WriteFileBytes(const std::filesystem::path& path, const std::vector<uint8_t>& bytes) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            return false;
        }
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out) {
            return false;
        }
        out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        return static_cast<bool>(out);
    }

    struct HttpWorkerContext {
        HINTERNET hConnect = nullptr;
        std::wstring host;
        INTERNET_PORT port = 0;

        ~HttpWorkerContext() {
            if (hConnect) {
                WinHttpCloseHandle(hConnect);
                hConnect = nullptr;
            }
        }
    };

    struct WicWorkerContext {
        ComPtr<IWICImagingFactory> factory;
    };

    bool HttpGetBytes(const std::string& urlUtf8, std::vector<uint8_t>& outBytes, HttpWorkerContext& ctx) {
        if (!hSession) {
            return false;
        }

        std::wstring url = Utf8ToWide(urlUtf8);
        if (url.empty()) {
            return false;
        }

        URL_COMPONENTS parts{};
        parts.dwStructSize = sizeof(parts);
        parts.dwSchemeLength = static_cast<DWORD>(-1);
        parts.dwHostNameLength = static_cast<DWORD>(-1);
        parts.dwUrlPathLength = static_cast<DWORD>(-1);
        parts.dwExtraInfoLength = static_cast<DWORD>(-1);
        if (!WinHttpCrackUrl(url.c_str(), static_cast<DWORD>(url.size()), 0, &parts)) {
            return false;
        }

        std::wstring host(parts.lpszHostName, parts.dwHostNameLength);
        std::wstring path(parts.lpszUrlPath, parts.dwUrlPathLength);
        if (parts.dwExtraInfoLength > 0) {
            path += std::wstring(parts.lpszExtraInfo, parts.dwExtraInfoLength);
        }
        if (path.empty()) {
            path = L"/";
        }

        const bool isHttps = (parts.nScheme == INTERNET_SCHEME_HTTPS);
        const DWORD openFlags = isHttps ? WINHTTP_FLAG_SECURE : 0;
        const bool connectionChanged =
            (!ctx.hConnect) || (ctx.port != parts.nPort) || (ctx.host != host);
        if (connectionChanged) {
            if (ctx.hConnect) {
                WinHttpCloseHandle(ctx.hConnect);
                ctx.hConnect = nullptr;
            }
            ctx.hConnect = WinHttpConnect(hSession, host.c_str(), parts.nPort, 0);
            if (!ctx.hConnect) {
                return false;
            }
            ctx.host = host;
            ctx.port = parts.nPort;
        }

        HINTERNET hRequest = WinHttpOpenRequest(
            ctx.hConnect,
            L"GET",
            path.c_str(),
            nullptr,
            WINHTTP_NO_REFERER,
            WINHTTP_DEFAULT_ACCEPT_TYPES,
            openFlags);
        if (!hRequest) {
            return false;
        }

        bool ok = false;
        if (WinHttpSendRequest(hRequest, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) &&
            WinHttpReceiveResponse(hRequest, nullptr)) {
            DWORD statusCode = 0;
            DWORD statusSize = sizeof(statusCode);
            WinHttpQueryHeaders(
                hRequest,
                WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                WINHTTP_HEADER_NAME_BY_INDEX,
                &statusCode,
                &statusSize,
                WINHTTP_NO_HEADER_INDEX);
            if (statusCode == 200) {
                outBytes.clear();
                while (true) {
                    DWORD available = 0;
                    if (!WinHttpQueryDataAvailable(hRequest, &available)) {
                        break;
                    }
                    if (available == 0) {
                        ok = true;
                        break;
                    }
                    const size_t oldSize = outBytes.size();
                    outBytes.resize(oldSize + static_cast<size_t>(available));
                    DWORD readBytes = 0;
                    if (!WinHttpReadData(hRequest, outBytes.data() + oldSize, available, &readBytes)) {
                        ok = false;
                        break;
                    }
                    if (readBytes != available) {
                        outBytes.resize(oldSize + static_cast<size_t>(readBytes));
                    }
                }
            }
        }

        WinHttpCloseHandle(hRequest);
        return ok && !outBytes.empty();
    }

    bool EnsureWicWorkerContext(WicWorkerContext& ctx) {
        if (ctx.factory) {
            return true;
        }
        const HRESULT hr = CoCreateInstance(
            CLSID_WICImagingFactory,
            nullptr,
            CLSCTX_INPROC_SERVER,
            IID_PPV_ARGS(ctx.factory.ReleaseAndGetAddressOf()));
        return SUCCEEDED(hr);
    }

    bool DecodeJpegToRgba(const std::vector<uint8_t>& jpegBytes, std::vector<uint8_t>& outRgba, WicWorkerContext& ctx) {
        if (!EnsureWicWorkerContext(ctx)) {
            return false;
        }

        ComPtr<IWICStream> stream;
        HRESULT hr = ctx.factory->CreateStream(stream.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            return false;
        }
        hr = stream->InitializeFromMemory(const_cast<BYTE*>(jpegBytes.data()), static_cast<DWORD>(jpegBytes.size()));
        if (FAILED(hr)) {
            return false;
        }

        ComPtr<IWICBitmapDecoder> decoder;
        hr = ctx.factory->CreateDecoderFromStream(stream.Get(), nullptr, WICDecodeMetadataCacheOnLoad, decoder.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            return false;
        }

        ComPtr<IWICBitmapFrameDecode> frame;
        hr = decoder->GetFrame(0, frame.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            return false;
        }

        UINT width = 0;
        UINT height = 0;
        frame->GetSize(&width, &height);
        if (width == 0 || height == 0) {
            return false;
        }

        ComPtr<IWICFormatConverter> converter;
        hr = ctx.factory->CreateFormatConverter(converter.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            return false;
        }
        hr = converter->Initialize(
            frame.Get(),
            GUID_WICPixelFormat32bppRGBA,
            WICBitmapDitherTypeNone,
            nullptr,
            0.0,
            WICBitmapPaletteTypeCustom);
        if (FAILED(hr)) {
            return false;
        }

        const UINT stride = width * 4;
        outRgba.resize(static_cast<size_t>(stride) * static_cast<size_t>(height));
        hr = converter->CopyPixels(nullptr, stride, static_cast<UINT>(outRgba.size()), outRgba.data());
        if (FAILED(hr)) {
            return false;
        }
        return width == 256 && height == 256;
    }

    void TouchLru(const TileKey& key) {
        const auto found = tileCache.find(key);
        if (found == tileCache.end()) {
            return;
        }
        lru.splice(lru.begin(), lru, found->second.lruIt);
        found->second.lruIt = lru.begin();
    }

    void InsertDecodedTile(const TileKey& key, std::vector<uint8_t>&& rgba) {
        std::scoped_lock<std::mutex> lock(mutex);
        const auto existing = tileCache.find(key);
        if (existing != tileCache.end()) {
            TouchLru(key);
            return;
        }
        lru.push_front(key);
        CachedTile entry{};
        entry.rgbaPixels = std::make_shared<std::vector<uint8_t>>(std::move(rgba));
        entry.lruIt = lru.begin();
        tileCache.emplace(key, std::move(entry));

        while (tileCache.size() > config.maxMemoryTiles && !lru.empty()) {
            const TileKey tail = lru.back();
            lru.pop_back();
            tileCache.erase(tail);
        }
        if (dirtyTileSet.insert(key).second) {
            dirtyTileQueue.push_back(key);
        }
        dirtySinceLastCompose = true;
    }

    bool FetchAndDecodeTile(const TileKey& key, HttpWorkerContext& httpCtx, WicWorkerContext& wicCtx) {
        std::vector<uint8_t> bytes;
        const std::filesystem::path diskPath = CachePathForTile(key);
        bool haveBytes = ReadFileBytes(diskPath, bytes);
        if (haveBytes) {
            ++diskHits;
        } else {
            const std::string url = BuildTileUrl(key);
            if (!HttpGetBytes(url, bytes, httpCtx)) {
                ++networkFailures;
                return false;
            }
            ++networkHits;
            (void)WriteFileBytes(diskPath, bytes);
        }

        std::vector<uint8_t> rgba;
        if (!DecodeJpegToRgba(bytes, rgba, wicCtx)) {
            ++decodeFailures;
            return false;
        }

        InsertDecodedTile(key, std::move(rgba));
        return true;
    }

    void WorkerLoop() {
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        HttpWorkerContext httpCtx{};
        WicWorkerContext wicCtx{};
        while (true) {
            TileKey key{};
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [this]() { return stopWorkers || !requestQueue.empty(); });
                if (stopWorkers && requestQueue.empty()) {
                    break;
                }
                key = requestQueue.front();
                requestQueue.pop_front();
                queuedSet.erase(key);
                if (tileCache.find(key) != tileCache.end()) {
                    continue;
                }
            }
            (void)FetchAndDecodeTile(key, httpCtx, wicCtx);
        }
        CoUninitialize();
    }

    std::unordered_map<TileKey, std::shared_ptr<const std::vector<uint8_t>>, TileKeyHasher> CacheSnapshot() {
        std::unordered_map<TileKey, std::shared_ptr<const std::vector<uint8_t>>, TileKeyHasher> snapshot;
        std::scoped_lock<std::mutex> lock(mutex);
        snapshot.reserve(tileCache.size());
        for (const auto& [key, tile] : tileCache) {
            snapshot.emplace(key, tile.rgbaPixels);
        }
        return snapshot;
    }

    std::vector<TileKey> ConsumeDirtyTiles(size_t maxCount, bool& hasMore) {
        std::vector<TileKey> out;
        out.reserve(maxCount);
        std::scoped_lock<std::mutex> lock(mutex);
        while (!dirtyTileQueue.empty() && out.size() < maxCount) {
            const TileKey key = dirtyTileQueue.front();
            dirtyTileQueue.pop_front();
            dirtyTileSet.erase(key);
            out.push_back(key);
        }
        hasMore = !dirtyTileQueue.empty();
        return out;
    }

    bool SampleColorFromSnapshot(
        const std::unordered_map<TileKey, std::shared_ptr<const std::vector<uint8_t>>, TileKeyHasher>& snapshot,
        double latDeg,
        double lonDeg,
        int targetZoom,
        std::array<uint8_t, 4>& outColor,
        int& outSampleZoom) const {
        const double lat = ClampLatDeg(latDeg);
        const double lonWrapped = WrapLonDeg(lonDeg);
        outSampleZoom = -1;

        for (int z = targetZoom; z >= 0; --z) {
            const int n = 1 << z;
            const double x = LonToTileXUnwrapped(lonWrapped, z);
            const double y = LatToTileY(lat, z);
            const int xi = static_cast<int>(std::floor(x));
            const int yi = static_cast<int>(std::floor(y));
            const int xNorm = WrapTileX(xi, n);
            const int yClamp = std::clamp(yi, 0, n - 1);

            TileKey key{z, xNorm, yClamp};
            const auto found = snapshot.find(key);
            if (found == snapshot.end() || !found->second || found->second->size() < (256u * 256u * 4u)) {
                continue;
            }

            const double fx = x - std::floor(x);
            const double fy = y - std::floor(y);
            const double pxf = fx * 255.0;
            const double pyf = fy * 255.0;
            const int px0 = std::clamp(static_cast<int>(std::floor(pxf)), 0, 255);
            const int py0 = std::clamp(static_cast<int>(std::floor(pyf)), 0, 255);
            const int px1 = std::min(px0 + 1, 255);
            const int py1 = std::min(py0 + 1, 255);
            const float tx = static_cast<float>(pxf - static_cast<double>(px0));
            const float ty = static_cast<float>(pyf - static_cast<double>(py0));

            const auto sampleChannel = [&](int xPx, int yPx, int channel) -> float {
                const size_t idx = (static_cast<size_t>(yPx) * 256u + static_cast<size_t>(xPx)) * 4u + static_cast<size_t>(channel);
                return static_cast<float>((*found->second)[idx]);
            };
            for (int c = 0; c < 3; ++c) {
                const float c00 = sampleChannel(px0, py0, c);
                const float c10 = sampleChannel(px1, py0, c);
                const float c01 = sampleChannel(px0, py1, c);
                const float c11 = sampleChannel(px1, py1, c);
                const float c0 = c00 + (c10 - c00) * tx;
                const float c1 = c01 + (c11 - c01) * tx;
                const float value = c0 + (c1 - c0) * ty;
                outColor[c] = static_cast<uint8_t>(std::clamp(std::lround(value), 0l, 255l));
            }
            outColor[3] = 0;
            outSampleZoom = z;
            return true;
        }
        return false;
    }

    DirectX::XMFLOAT4 ComputeRingBounds(double centerLatDeg, double centerLonDeg, const RingParams& params) const {
        const double latRad = flight::DegToRad(centerLatDeg);
        const double radiusMeters = params.radiusKm * 1000.0;
        const double dLatDeg = flight::RadToDeg(radiusMeters / kEarthRadiusMeters);
        const double cosLat = std::max(0.15, std::cos(latRad));
        const double dLonDeg = flight::RadToDeg(radiusMeters / (kEarthRadiusMeters * cosLat));
        const double lonW = centerLonDeg - dLonDeg;
        const double lonE = centerLonDeg + dLonDeg;
        const double latS = std::clamp(centerLatDeg - dLatDeg, -85.0, 85.0);
        const double latN = std::clamp(centerLatDeg + dLatDeg, -85.0, 85.0);
        return {
            static_cast<float>(lonW),
            static_cast<float>(lonE),
            static_cast<float>(latS),
            static_cast<float>(latN),
        };
    }

    bool DirtyTileToPixelRect(
        const RingTexture& ring,
        const TileKey& key,
        uint32_t& outX0,
        uint32_t& outY0,
        uint32_t& outX1Exclusive,
        uint32_t& outY1Exclusive) const {
        if (ring.width == 0 || ring.height == 0 || ring.boundsLonLat.x >= ring.boundsLonLat.y ||
            ring.boundsLonLat.z >= ring.boundsLonLat.w) {
            return false;
        }

        const int n = 1 << key.z;
        const double tileLonWBase = (static_cast<double>(key.x) / static_cast<double>(n)) * 360.0 - 180.0;
        const double tileLonEBase = (static_cast<double>(key.x + 1) / static_cast<double>(n)) * 360.0 - 180.0;
        const double tileLatN = TileYToLatDeg(static_cast<double>(key.y), key.z);
        const double tileLatS = TileYToLatDeg(static_cast<double>(key.y + 1), key.z);
        const double tileMercN = LatDegToMercatorY(tileLatN);
        const double tileMercS = LatDegToMercatorY(tileLatS);

        const double ringLonW = ring.boundsLonLat.x;
        const double ringLonE = ring.boundsLonLat.y;
        const double ringLatS = ring.boundsLonLat.z;
        const double ringLatN = ring.boundsLonLat.w;
        const double ringMercS = LatDegToMercatorY(ringLatS);
        const double ringMercN = LatDegToMercatorY(ringLatN);
        const double ringLonCenter = 0.5 * (ringLonW + ringLonE);
        const double tileLonCenterBase = 0.5 * (tileLonWBase + tileLonEBase);
        const double wrapShift = std::round((ringLonCenter - tileLonCenterBase) / 360.0) * 360.0;
        const double tileLonW = tileLonWBase + wrapShift;
        const double tileLonE = tileLonEBase + wrapShift;

        const double interLonW = std::max(ringLonW, tileLonW);
        const double interLonE = std::min(ringLonE, tileLonE);
        const double interMercS = std::max(ringMercS, tileMercS);
        const double interMercN = std::min(ringMercN, tileMercN);
        if (interLonW >= interLonE || interMercS >= interMercN) {
            return false;
        }

        const double lonSpan = ringLonE - ringLonW;
        const double mercSpan = ringMercN - ringMercS;
        if (lonSpan <= 1e-9 || mercSpan <= 1e-9) {
            return false;
        }

        constexpr int kPad = 2;
        const double u0 = (interLonW - ringLonW) / lonSpan;
        const double u1 = (interLonE - ringLonW) / lonSpan;
        const double v0 = (ringMercN - interMercN) / mercSpan;
        const double v1 = (ringMercN - interMercS) / mercSpan;

        int x0 = static_cast<int>(std::floor(u0 * static_cast<double>(ring.width))) - kPad;
        int x1 = static_cast<int>(std::ceil(u1 * static_cast<double>(ring.width))) + kPad;
        int y0 = static_cast<int>(std::floor(v0 * static_cast<double>(ring.height))) - kPad;
        int y1 = static_cast<int>(std::ceil(v1 * static_cast<double>(ring.height))) + kPad;

        x0 = std::clamp(x0, 0, static_cast<int>(ring.width));
        x1 = std::clamp(x1, 0, static_cast<int>(ring.width));
        y0 = std::clamp(y0, 0, static_cast<int>(ring.height));
        y1 = std::clamp(y1, 0, static_cast<int>(ring.height));
        if (x0 >= x1 || y0 >= y1) {
            return false;
        }

        outX0 = static_cast<uint32_t>(x0);
        outY0 = static_cast<uint32_t>(y0);
        outX1Exclusive = static_cast<uint32_t>(x1);
        outY1Exclusive = static_cast<uint32_t>(y1);
        return true;
    }

    void UpdateRingPixels(
        RingTexture& ring,
        const RingParams& params,
        const std::unordered_map<TileKey, std::shared_ptr<const std::vector<uint8_t>>, TileKeyHasher>& snapshot,
        uint32_t xBegin,
        uint32_t yBegin,
        uint32_t xEndExclusive,
        uint32_t yEndExclusive) const {
        if (xBegin >= xEndExclusive || yBegin >= yEndExclusive || ring.width == 0 || ring.height == 0) {
            return;
        }
        const double lonW = ring.boundsLonLat.x;
        const double lonE = ring.boundsLonLat.y;
        const double latS = ring.boundsLonLat.z;
        const double latN = ring.boundsLonLat.w;
        const double mercS = LatDegToMercatorY(latS);
        const double mercN = LatDegToMercatorY(latN);
        const double invW = 1.0 / static_cast<double>(ring.width);
        const double invH = 1.0 / static_cast<double>(ring.height);
        constexpr double kEdgeFeather = 0.08;
        for (uint32_t y = yBegin; y < yEndExclusive; ++y) {
            const double v = (static_cast<double>(y) + 0.5) * invH;
            const double merc = mercN + (mercS - mercN) * v;
            const double lat = MercatorYToLatDeg(merc);
            for (uint32_t x = xBegin; x < xEndExclusive; ++x) {
                const double u = (static_cast<double>(x) + 0.5) * invW;
                const double lon = lonW + (lonE - lonW) * u;
                std::array<uint8_t, 4> color{0, 0, 0, 0};
                int sampledZoom = -1;
                if (SampleColorFromSnapshot(snapshot, lat, lon, params.zoom, color, sampledZoom)) {
                    const int zoomDelta = std::max(0, params.zoom - sampledZoom);
                    float lodQuality = 1.0f - static_cast<float>(zoomDelta) * 0.18f;
                    lodQuality = std::clamp(lodQuality, 0.30f, 1.0f);
                    const double edgeU = std::min(u, 1.0 - u);
                    const double edgeV = std::min(v, 1.0 - v);
                    const double edgeDist = std::min(edgeU, edgeV);
                    const float edgeFade = static_cast<float>(std::clamp(edgeDist / kEdgeFeather, 0.0, 1.0));
                    const float alphaF = std::clamp(lodQuality * edgeFade, 0.0f, 1.0f);
                    color[3] = static_cast<uint8_t>(std::round(alphaF * 255.0f));
                }
                const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(ring.width) + static_cast<size_t>(x)) * 4u;
                ring.rgbaPixels[idx + 0] = color[0];
                ring.rgbaPixels[idx + 1] = color[1];
                ring.rgbaPixels[idx + 2] = color[2];
                ring.rgbaPixels[idx + 3] = color[3];
            }
        }
    }

    bool ComposeRing(
        double centerLatDeg,
        double centerLonDeg,
        const RingParams& params,
        const std::unordered_map<TileKey, std::shared_ptr<const std::vector<uint8_t>>, TileKeyHasher>& snapshot,
        const std::vector<TileKey>* dirtyTiles,
        const RingTexture* previous,
        RingTexture& out) {
        out.zoom = params.zoom;
        out.width = static_cast<uint32_t>(params.textureSize);
        out.height = static_cast<uint32_t>(params.textureSize);
        out.boundsLonLat = ComputeRingBounds(centerLatDeg, centerLonDeg, params);

        const bool canIncremental =
            dirtyTiles &&
            previous &&
            previous->zoom == out.zoom &&
            previous->width == out.width &&
            previous->height == out.height &&
            previous->rgbaPixels.size() == static_cast<size_t>(out.width) * static_cast<size_t>(out.height) * 4u &&
            std::abs(previous->boundsLonLat.x - out.boundsLonLat.x) < 1e-5f &&
            std::abs(previous->boundsLonLat.y - out.boundsLonLat.y) < 1e-5f &&
            std::abs(previous->boundsLonLat.z - out.boundsLonLat.z) < 1e-5f &&
            std::abs(previous->boundsLonLat.w - out.boundsLonLat.w) < 1e-5f;

        if (canIncremental) {
            out.rgbaPixels = previous->rgbaPixels;
            for (const TileKey& key : *dirtyTiles) {
                uint32_t x0 = 0;
                uint32_t y0 = 0;
                uint32_t x1 = 0;
                uint32_t y1 = 0;
                if (!DirtyTileToPixelRect(out, key, x0, y0, x1, y1)) {
                    continue;
                }
                UpdateRingPixels(out, params, snapshot, x0, y0, x1, y1);
            }
        } else {
            out.rgbaPixels.assign(static_cast<size_t>(out.width) * static_cast<size_t>(out.height) * 4u, 0u);
            UpdateRingPixels(out, params, snapshot, 0, 0, out.width, out.height);
        }

        bool hasAlpha = false;
        for (size_t idx = 3; idx < out.rgbaPixels.size(); idx += 4) {
            if (out.rgbaPixels[idx] > 0) {
                hasAlpha = true;
                break;
            }
        }
        out.valid = hasAlpha;
        return true;
    }

    bool ComposeLodRings(
        double latDeg,
        double lonDeg,
        double altitudeMeters,
        bool force,
        std::array<RingTexture, 3>& outRings,
        std::string& error) {
        (void)error;
        if (!initialized) {
            return false;
        }

        auto pickCenter = [](double sourceLatDeg, double sourceLonDeg, double snapMeters, double& outLatDeg, double& outLonDeg) {
            const double stepLatDeg = flight::RadToDeg(snapMeters / kEarthRadiusMeters);
            outLatDeg = std::clamp(SnapToStep(sourceLatDeg, stepLatDeg), -85.0, 85.0);
            const double cosLat = std::max(0.20, std::cos(flight::DegToRad(outLatDeg)));
            const double stepLonDeg = flight::RadToDeg(snapMeters / (kEarthRadiusMeters * cosLat));
            outLonDeg = WrapLonDeg(SnapToStep(sourceLonDeg, stepLonDeg));
        };

        const auto targetRings = ComputeRingParams(altitudeMeters);
        RingParams composeRings[3] = {targetRings[0], targetRings[1], targetRings[2]};

        const bool dirty = dirtySinceLastCompose.load();
        const bool hasNearState = hasComposeState && hasComposedRing[0];
        const bool hasMidState = hasComposeState && hasComposedRing[1];
        const bool hasFarState = hasComposeState && hasComposedRing[2];
        const bool hasMidFarState = hasMidState && hasFarState;

        const bool nearZoomTargetChanged = !hasNearState || (lastComposeZooms[0] != targetRings[0].zoom);
        const bool midZoomChanged = !hasMidState || (lastComposeZooms[1] != targetRings[1].zoom);
        const bool farZoomChanged = !hasFarState || (lastComposeZooms[2] != targetRings[2].zoom);
        const bool nearRadiusChanged =
            !hasNearState || std::abs(lastComposeRadiusKm[0] - targetRings[0].radiusKm) > std::max(5.0, targetRings[0].radiusKm * 0.20);
        const bool midRadiusChanged =
            !hasMidState || std::abs(lastComposeRadiusKm[1] - targetRings[1].radiusKm) > std::max(8.0, targetRings[1].radiusKm * 0.15);
        const bool farRadiusChanged =
            !hasFarState || std::abs(lastComposeRadiusKm[2] - targetRings[2].radiusKm) > std::max(15.0, targetRings[2].radiusKm * 0.12);

        const double nearRecenterMeters = std::clamp(targetRings[0].radiusKm * 1000.0 * 0.65, 9000.0, 65000.0);
        const double nearSnapMeters = std::clamp(nearRecenterMeters * 0.30, 3000.0, 22000.0);
        const double midFarRecenterMeters = std::clamp(targetRings[1].radiusKm * 1000.0 * 0.30, 3500.0, 32000.0);

        const bool nearMoved =
            !hasNearState || DistanceMeters(lastNearComposeLatDeg, lastNearComposeLonDeg, latDeg, lonDeg) > nearRecenterMeters;
        const bool midFarMoved =
            !hasMidFarState || DistanceMeters(lastMidFarComposeLatDeg, lastMidFarComposeLonDeg, latDeg, lonDeg) > midFarRecenterMeters;

        const bool allowNearZoomChange = force || nearMoved || nearRadiusChanged || !hasNearState;
        if (!allowNearZoomChange && hasNearState) {
            composeRings[0].zoom = lastComposeZooms[0];
        }

        const bool needsNearCompose =
            force || !hasNearState || dirty || nearMoved || nearRadiusChanged || (nearZoomTargetChanged && allowNearZoomChange);
        const bool needsMidFarCompose =
            force || !hasMidFarState || dirty || nearMoved || midFarMoved || midZoomChanged || farZoomChanged || midRadiusChanged ||
            farRadiusChanged;

        if (!needsNearCompose && !needsMidFarCompose) {
            return false;
        }

        double nearLatDeg = latDeg;
        double nearLonDeg = WrapLonDeg(lonDeg);
        if (hasNearState && !force && !nearMoved) {
            nearLatDeg = lastNearComposeLatDeg;
            nearLonDeg = lastNearComposeLonDeg;
        } else {
            pickCenter(latDeg, lonDeg, nearSnapMeters, nearLatDeg, nearLonDeg);
        }

        double midFarLatDeg = nearLatDeg;
        double midFarLonDeg = nearLonDeg;

        bool dirtyTilesRemaining = false;
        std::vector<TileKey> dirtyTilesForCompose;
        if (dirty) {
            size_t dirtyBacklog = 0;
            {
                std::scoped_lock<std::mutex> lock(mutex);
                dirtyBacklog = dirtyTileQueue.size();
            }
            const size_t dirtyBatchBudget = std::clamp<size_t>(dirtyBacklog + 128, 384, 2048);
            dirtyTilesForCompose = ConsumeDirtyTiles(dirtyBatchBudget, dirtyTilesRemaining);
        }
        const std::vector<TileKey>* dirtyTilesPtr = dirtyTilesForCompose.empty() ? nullptr : &dirtyTilesForCompose;
        const auto snapshot = CacheSnapshot();
        if (needsNearCompose) {
            const RingTexture* prevNear = hasComposedRing[0] ? &lastComposedRings[0] : nullptr;
            ComposeRing(
                nearLatDeg,
                nearLonDeg,
                composeRings[0],
                snapshot,
                dirtyTilesPtr,
                prevNear,
                outRings[0]);
            lastComposedRings[0] = outRings[0];
            hasComposedRing[0] = true;
            lastNearComposeLatDeg = nearLatDeg;
            lastNearComposeLonDeg = nearLonDeg;
            lastComposeZooms[0] = composeRings[0].zoom;
            lastComposeRadiusKm[0] = composeRings[0].radiusKm;
        } else {
            outRings[0] = lastComposedRings[0];
        }

        if (needsMidFarCompose) {
            const RingTexture* prevMid = hasComposedRing[1] ? &lastComposedRings[1] : nullptr;
            const RingTexture* prevFar = hasComposedRing[2] ? &lastComposedRings[2] : nullptr;
            ComposeRing(
                midFarLatDeg,
                midFarLonDeg,
                composeRings[1],
                snapshot,
                dirtyTilesPtr,
                prevMid,
                outRings[1]);
            ComposeRing(
                midFarLatDeg,
                midFarLonDeg,
                composeRings[2],
                snapshot,
                dirtyTilesPtr,
                prevFar,
                outRings[2]);
            lastComposedRings[1] = outRings[1];
            lastComposedRings[2] = outRings[2];
            hasComposedRing[1] = true;
            hasComposedRing[2] = true;
            lastMidFarComposeLatDeg = midFarLatDeg;
            lastMidFarComposeLonDeg = midFarLonDeg;
            lastComposeZooms[1] = composeRings[1].zoom;
            lastComposeZooms[2] = composeRings[2].zoom;
            lastComposeRadiusKm[1] = composeRings[1].radiusKm;
            lastComposeRadiusKm[2] = composeRings[2].radiusKm;
        } else {
            outRings[1] = lastComposedRings[1];
            outRings[2] = lastComposedRings[2];
        }

        hasComposeState = true;
        lastComposeLatDeg = nearLatDeg;
        lastComposeLonDeg = nearLonDeg;
        bool stillDirty = dirtyTilesRemaining;
        if (!stillDirty) {
            std::scoped_lock<std::mutex> lock(mutex);
            stillDirty = !dirtyTileQueue.empty();
        }
        dirtySinceLastCompose = stillDirty;
        return true;
    }

    bool QueueTileRequest(int zoom, int tileX, int tileY, bool highPriority) {
        if (!initialized || zoom < 0 || zoom > 22) {
            return false;
        }
        const int n = 1 << zoom;
        TileKey key{};
        key.z = zoom;
        key.x = WrapTileX(tileX, n);
        key.y = std::clamp(tileY, 0, n - 1);
        return EnqueueTileRequest(key, highPriority);
    }

    bool TryGetCachedTileRgba(
        int zoom,
        int tileX,
        int tileY,
        std::shared_ptr<const std::vector<uint8_t>>& outPixels) {
        outPixels.reset();
        if (!initialized || zoom < 0 || zoom > 22) {
            return false;
        }
        const int n = 1 << zoom;
        TileKey key{};
        key.z = zoom;
        key.x = WrapTileX(tileX, n);
        key.y = std::clamp(tileY, 0, n - 1);

        std::scoped_lock<std::mutex> lock(mutex);
        const auto found = tileCache.find(key);
        if (found == tileCache.end() || !found->second.rgbaPixels) {
            return false;
        }
        TouchLru(key);
        outPixels = found->second.rgbaPixels;
        return true;
    }

    StreamStats Stats() const {
        StreamStats s{};
        s.diskHits = diskHits.load();
        s.networkHits = networkHits.load();
        s.networkFailures = networkFailures.load();
        s.decodeFailures = decodeFailures.load();
        s.activeZooms = activeZooms;
        {
            std::scoped_lock<std::mutex> lock(mutex);
            s.memoryTileCount = tileCache.size();
            s.queuedRequests = requestQueue.size();
        }
        return s;
    }
};

SatelliteStreamer::~SatelliteStreamer() {
    Shutdown();
}

bool SatelliteStreamer::Initialize(const Config& config, std::string& error) {
    Shutdown();
    m_impl = new Impl();
    if (!m_impl->Initialize(config, error)) {
        delete m_impl;
        m_impl = nullptr;
        return false;
    }
    return true;
}

void SatelliteStreamer::Shutdown() {
    if (!m_impl) {
        return;
    }
    m_impl->Shutdown();
    delete m_impl;
    m_impl = nullptr;
}

void SatelliteStreamer::PrefetchForView(double latDeg, double lonDeg, double altitudeMeters, double headingDeg) {
    if (!m_impl) {
        return;
    }
    m_impl->PrefetchForView(latDeg, lonDeg, altitudeMeters, headingDeg);
}

bool SatelliteStreamer::ComposeLodRings(
    double latDeg,
    double lonDeg,
    double altitudeMeters,
    bool force,
    std::array<RingTexture, 3>& outRings,
    std::string& error) {
    if (!m_impl) {
        error = "SatelliteStreamer is not initialized";
        return false;
    }
    return m_impl->ComposeLodRings(latDeg, lonDeg, altitudeMeters, force, outRings, error);
}

bool SatelliteStreamer::QueueTileRequest(int zoom, int tileX, int tileY, bool highPriority) {
    if (!m_impl) {
        return false;
    }
    return m_impl->QueueTileRequest(zoom, tileX, tileY, highPriority);
}

bool SatelliteStreamer::TryGetCachedTileRgba(
    int zoom,
    int tileX,
    int tileY,
    std::shared_ptr<const std::vector<uint8_t>>& outPixels) {
    if (!m_impl) {
        outPixels.reset();
        return false;
    }
    return m_impl->TryGetCachedTileRgba(zoom, tileX, tileY, outPixels);
}

StreamStats SatelliteStreamer::GetStats() const {
    if (!m_impl) {
        return {};
    }
    return m_impl->Stats();
}

} // namespace flight::satellite
