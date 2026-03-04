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

    std::atomic<uint64_t> diskHits{0};
    std::atomic<uint64_t> networkHits{0};
    std::atomic<uint64_t> networkFailures{0};
    std::atomic<uint64_t> decodeFailures{0};

    std::atomic<bool> dirtySinceLastCompose{true};
    std::array<int, 3> activeZooms{13, 11, 9};
    double lastComposeLatDeg = 1000.0;
    double lastComposeLonDeg = 1000.0;
    std::array<int, 3> lastComposeZooms{-1, -1, -1};
    bool hasComposeState = false;

    HINTERNET hSession = nullptr;

    ~Impl() {
        Shutdown();
    }

    bool Initialize(const Config& cfg, std::string& error) {
        Shutdown();
        config = cfg;
        config.workerCount = std::clamp(config.workerCount, 1, 8);
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
        }
        hasComposeState = false;
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

        // Scale stream coverage with altitude so cruising flight still sees streamed imagery instead of only fallback albedo.
        const double farRadiusKm = std::clamp(horizonKm * 0.65, 80.0, 520.0);
        const double midRadiusKm = std::clamp(farRadiusKm * 0.32, 20.0, 220.0);
        const double nearRadiusKm = std::clamp(midRadiusKm * 0.26, 4.0, 70.0);

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

    void EnqueueTileRequest(const TileKey& key, bool highPriority) {
        std::scoped_lock<std::mutex> lock(mutex);
        if (tileCache.find(key) != tileCache.end()) {
            return;
        }
        if (queuedSet.find(key) != queuedSet.end()) {
            return;
        }
        if (highPriority) {
            requestQueue.push_front(key);
        } else {
            requestQueue.push_back(key);
        }
        queuedSet.insert(key);
        cv.notify_one();
    }

    void QueueBounds(int zoom, double latDeg, double lonDeg, double radiusKm, int capRadiusTiles, bool highPriority) {
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

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                TileKey key{};
                key.z = zoom;
                key.x = WrapTileX(x, n);
                key.y = std::clamp(y, 0, n - 1);
                EnqueueTileRequest(key, highPriority);
            }
        }
    }

    void PrefetchForView(double latDeg, double lonDeg, double altitudeMeters) {
        if (!initialized) {
            return;
        }
        const auto rings = ComputeRingParams(altitudeMeters);
        size_t queued = 0;
        {
            std::scoped_lock<std::mutex> lock(mutex);
            queued = requestQueue.size();
        }
        for (int i = 0; i < 3; ++i) {
            if (i > 0 && queued > 3000) {
                break;
            }
            QueueBounds(
                rings[i].zoom,
                latDeg,
                lonDeg,
                rings[i].radiusKm,
                rings[i].prefetchRadiusTiles,
                i == 0);
            if (queued < 6000) {
                queued += 256;
            }
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

    bool HttpGetBytes(const std::string& urlUtf8, std::vector<uint8_t>& outBytes) {
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

        HINTERNET hConnect = WinHttpConnect(hSession, host.c_str(), parts.nPort, 0);
        if (!hConnect) {
            return false;
        }

        HINTERNET hRequest = WinHttpOpenRequest(
            hConnect,
            L"GET",
            path.c_str(),
            nullptr,
            WINHTTP_NO_REFERER,
            WINHTTP_DEFAULT_ACCEPT_TYPES,
            openFlags);
        if (!hRequest) {
            WinHttpCloseHandle(hConnect);
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
        WinHttpCloseHandle(hConnect);
        return ok && !outBytes.empty();
    }

    bool DecodeJpegToRgba(const std::vector<uint8_t>& jpegBytes, std::vector<uint8_t>& outRgba) {
        ComPtr<IWICImagingFactory> factory;
        HRESULT hr = CoCreateInstance(
            CLSID_WICImagingFactory,
            nullptr,
            CLSCTX_INPROC_SERVER,
            IID_PPV_ARGS(factory.ReleaseAndGetAddressOf()));
        if (FAILED(hr)) {
            return false;
        }

        ComPtr<IWICStream> stream;
        hr = factory->CreateStream(stream.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            return false;
        }
        hr = stream->InitializeFromMemory(const_cast<BYTE*>(jpegBytes.data()), static_cast<DWORD>(jpegBytes.size()));
        if (FAILED(hr)) {
            return false;
        }

        ComPtr<IWICBitmapDecoder> decoder;
        hr = factory->CreateDecoderFromStream(stream.Get(), nullptr, WICDecodeMetadataCacheOnLoad, decoder.ReleaseAndGetAddressOf());
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
        hr = factory->CreateFormatConverter(converter.ReleaseAndGetAddressOf());
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
        dirtySinceLastCompose = true;
    }

    bool FetchAndDecodeTile(const TileKey& key) {
        std::vector<uint8_t> bytes;
        const std::filesystem::path diskPath = CachePathForTile(key);
        bool haveBytes = ReadFileBytes(diskPath, bytes);
        if (haveBytes) {
            ++diskHits;
        } else {
            const std::string url = BuildTileUrl(key);
            if (!HttpGetBytes(url, bytes)) {
                ++networkFailures;
                return false;
            }
            ++networkHits;
            (void)WriteFileBytes(diskPath, bytes);
        }

        std::vector<uint8_t> rgba;
        if (!DecodeJpegToRgba(bytes, rgba)) {
            ++decodeFailures;
            return false;
        }

        InsertDecodedTile(key, std::move(rgba));
        return true;
    }

    void WorkerLoop() {
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
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
            (void)FetchAndDecodeTile(key);
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

    bool SampleColorFromSnapshot(
        const std::unordered_map<TileKey, std::shared_ptr<const std::vector<uint8_t>>, TileKeyHasher>& snapshot,
        double latDeg,
        double lonDeg,
        int targetZoom,
        std::array<uint8_t, 4>& outColor) {
        const double lat = ClampLatDeg(latDeg);
        const double lonWrapped = WrapLonDeg(lonDeg);

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
            const int px = std::clamp(static_cast<int>(fx * 255.0), 0, 255);
            const int py = std::clamp(static_cast<int>(fy * 255.0), 0, 255);
            const size_t idx = (static_cast<size_t>(py) * 256u + static_cast<size_t>(px)) * 4u;
            outColor[0] = (*found->second)[idx + 0];
            outColor[1] = (*found->second)[idx + 1];
            outColor[2] = (*found->second)[idx + 2];
            outColor[3] = 255;
            return true;
        }
        return false;
    }

    RingTexture ComposeRing(
        double centerLatDeg,
        double centerLonDeg,
        const RingParams& params,
        const std::unordered_map<TileKey, std::shared_ptr<const std::vector<uint8_t>>, TileKeyHasher>& snapshot) {
        RingTexture out{};
        out.zoom = params.zoom;
        out.width = static_cast<uint32_t>(params.textureSize);
        out.height = static_cast<uint32_t>(params.textureSize);
        out.rgbaPixels.resize(static_cast<size_t>(out.width) * static_cast<size_t>(out.height) * 4u, 0u);

        const double latRad = flight::DegToRad(centerLatDeg);
        const double radiusMeters = params.radiusKm * 1000.0;
        const double dLatDeg = flight::RadToDeg(radiusMeters / kEarthRadiusMeters);
        const double cosLat = std::max(0.15, std::cos(latRad));
        const double dLonDeg = flight::RadToDeg(radiusMeters / (kEarthRadiusMeters * cosLat));

        const double lonW = centerLonDeg - dLonDeg;
        const double lonE = centerLonDeg + dLonDeg;
        const double latS = std::clamp(centerLatDeg - dLatDeg, -85.0, 85.0);
        const double latN = std::clamp(centerLatDeg + dLatDeg, -85.0, 85.0);
        out.boundsLonLat = {
            static_cast<float>(lonW),
            static_cast<float>(lonE),
            static_cast<float>(latS),
            static_cast<float>(latN),
        };

        const double invW = 1.0 / static_cast<double>(out.width);
        const double invH = 1.0 / static_cast<double>(out.height);
        size_t validPixelCount = 0;
        for (uint32_t y = 0; y < out.height; ++y) {
            const double v = (static_cast<double>(y) + 0.5) * invH;
            const double lat = latN + (latS - latN) * v;
            for (uint32_t x = 0; x < out.width; ++x) {
                const double u = (static_cast<double>(x) + 0.5) * invW;
                const double lon = lonW + (lonE - lonW) * u;
                std::array<uint8_t, 4> color{0, 0, 0, 0};
                if (SampleColorFromSnapshot(snapshot, lat, lon, params.zoom, color)) {
                    validPixelCount += 1;
                }
                const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(out.width) + static_cast<size_t>(x)) * 4u;
                out.rgbaPixels[idx + 0] = color[0];
                out.rgbaPixels[idx + 1] = color[1];
                out.rgbaPixels[idx + 2] = color[2];
                out.rgbaPixels[idx + 3] = color[3];
            }
        }

        out.valid = (validPixelCount > 0u);
        return out;
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

        const auto rings = ComputeRingParams(altitudeMeters);
        bool zoomChanged = false;
        if (hasComposeState) {
            for (int i = 0; i < 3; ++i) {
                if (lastComposeZooms[i] != rings[i].zoom) {
                    zoomChanged = true;
                    break;
                }
            }
        } else {
            zoomChanged = true;
        }

        bool movedEnough = true;
        if (hasComposeState) {
            const double d = DistanceMeters(lastComposeLatDeg, lastComposeLonDeg, latDeg, lonDeg);
            movedEnough = d > 2500.0;
        }

        const bool dirty = dirtySinceLastCompose.load();
        if (!force && !zoomChanged && !dirty && !movedEnough) {
            return false;
        }

        const auto snapshot = CacheSnapshot();
        outRings[0] = ComposeRing(latDeg, lonDeg, rings[0], snapshot);
        outRings[1] = ComposeRing(latDeg, lonDeg, rings[1], snapshot);
        outRings[2] = ComposeRing(latDeg, lonDeg, rings[2], snapshot);

        hasComposeState = true;
        lastComposeLatDeg = latDeg;
        lastComposeLonDeg = lonDeg;
        lastComposeZooms = {rings[0].zoom, rings[1].zoom, rings[2].zoom};
        dirtySinceLastCompose = false;
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

void SatelliteStreamer::PrefetchForView(double latDeg, double lonDeg, double altitudeMeters) {
    if (!m_impl) {
        return;
    }
    m_impl->PrefetchForView(latDeg, lonDeg, altitudeMeters);
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

StreamStats SatelliteStreamer::GetStats() const {
    if (!m_impl) {
        return {};
    }
    return m_impl->Stats();
}

} // namespace flight::satellite
