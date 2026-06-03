#include "hud_map_tile_streamer.h"

#include <windows.h>
#include <winhttp.h>
#include <wincodec.h>
#include <wrl/client.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace flight::hud {
namespace {

using Microsoft::WRL::ComPtr;

int WrapTileX(int x, int n) {
    int r = x % n;
    if (r < 0) {
        r += n;
    }
    return r;
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

struct HudMapTileStreamer::Impl {
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

    struct QueuedRequestState {
        uint64_t touchSerial = 0;
        bool highPriority = false;
    };

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

    Config config{};
    bool initialized = false;
    bool stopWorkers = false;
    std::vector<std::thread> workers;
    mutable std::mutex mutex;
    std::condition_variable cv;
    std::deque<TileKey> requestQueue;
    std::unordered_map<TileKey, QueuedRequestState, TileKeyHasher> queuedRequests;
    uint64_t requestTouchSerial = 0;

    std::unordered_map<TileKey, CachedTile, TileKeyHasher> tileCache;
    std::list<TileKey> lru;

    std::atomic<uint64_t> diskHits{0};
    std::atomic<uint64_t> networkHits{0};
    std::atomic<uint64_t> networkFailures{0};
    std::atomic<uint64_t> decodeFailures{0};

    HINTERNET hSession = nullptr;

    ~Impl() {
        Shutdown();
    }

    bool Initialize(const Config& cfg, std::string& error) {
        Shutdown();
        config = cfg;
        config.workerCount = std::clamp(config.workerCount, 1, 8);
        config.maxMemoryTiles = std::max<size_t>(config.maxMemoryTiles, 128);

        std::error_code ec;
        std::filesystem::create_directories(config.diskCacheDirectory, ec);
        if (ec) {
            error = "Failed creating HUD map cache directory: " + ec.message();
            return false;
        }

        hSession = WinHttpOpen(
            L"3DFlight/1.0 HUD map",
            WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY,
            WINHTTP_NO_PROXY_NAME,
            WINHTTP_NO_PROXY_BYPASS,
            0);
        if (!hSession) {
            error = "WinHttpOpen failed";
            return false;
        }
        DWORD maxConnections = static_cast<DWORD>(std::clamp(config.workerCount * 2, 4, 12));
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
            queuedRequests.clear();
        }
        cv.notify_all();
        for (auto& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
        workers.clear();
        {
            std::scoped_lock<std::mutex> lock(mutex);
            tileCache.clear();
            lru.clear();
            requestTouchSerial = 0;
            stopWorkers = false;
        }
        if (hSession) {
            WinHttpCloseHandle(hSession);
            hSession = nullptr;
        }
        initialized = false;
    }

    bool EnqueueTileRequest(const TileKey& key, bool highPriority) {
        std::scoped_lock<std::mutex> lock(mutex);
        if (tileCache.find(key) != tileCache.end()) {
            TouchLru(key);
            return false;
        }
        const uint64_t touchSerial = ++requestTouchSerial;
        const auto queuedIt = queuedRequests.find(key);
        if (queuedIt != queuedRequests.end()) {
            queuedIt->second.touchSerial = touchSerial;
            if (highPriority && !queuedIt->second.highPriority) {
                queuedIt->second.highPriority = true;
                const auto queueIt = std::find(requestQueue.begin(), requestQueue.end(), key);
                if (queueIt != requestQueue.end() && queueIt != requestQueue.begin()) {
                    const TileKey promoted = *queueIt;
                    requestQueue.erase(queueIt);
                    requestQueue.push_front(promoted);
                }
                cv.notify_one();
            }
            return false;
        }
        if (highPriority) {
            requestQueue.push_front(key);
        } else {
            requestQueue.push_back(key);
        }
        queuedRequests.emplace(key, QueuedRequestState{touchSerial, highPriority});
        cv.notify_one();
        return true;
    }

    std::filesystem::path CachePathForTile(const TileKey& key) const {
        return config.diskCacheDirectory / std::to_string(key.z) / std::to_string(key.x) / (std::to_string(key.y) + ".tile");
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

    bool HttpGetBytes(const std::string& urlUtf8, std::vector<uint8_t>& outBytes, HttpWorkerContext& ctx) {
        if (!hSession) {
            return false;
        }

        const std::wstring url = Utf8ToWide(urlUtf8);
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
        const bool connectionChanged = (!ctx.hConnect) || (ctx.port != parts.nPort) || (ctx.host != host);
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

    bool DecodeImageToRgba256(const std::vector<uint8_t>& bytes, std::vector<uint8_t>& outRgba, WicWorkerContext& ctx) {
        if (!EnsureWicWorkerContext(ctx)) {
            return false;
        }

        ComPtr<IWICStream> stream;
        HRESULT hr = ctx.factory->CreateStream(stream.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            return false;
        }
        hr = stream->InitializeFromMemory(const_cast<BYTE*>(bytes.data()), static_cast<DWORD>(bytes.size()));
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

        ComPtr<IWICBitmapSource> source;
        hr = frame.As(&source);
        if (FAILED(hr)) {
            return false;
        }
        ComPtr<IWICBitmapScaler> scaler;
        if (width != 256 || height != 256) {
            hr = ctx.factory->CreateBitmapScaler(scaler.ReleaseAndGetAddressOf());
            if (FAILED(hr)) {
                return false;
            }
            hr = scaler->Initialize(frame.Get(), 256, 256, WICBitmapInterpolationModeFant);
            if (FAILED(hr)) {
                return false;
            }
            hr = scaler.As(&source);
            if (FAILED(hr)) {
                return false;
            }
            width = 256;
            height = 256;
        }

        ComPtr<IWICFormatConverter> converter;
        hr = ctx.factory->CreateFormatConverter(converter.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            return false;
        }
        hr = converter->Initialize(
            source.Get(),
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
        return SUCCEEDED(hr) && outRgba.size() == (256u * 256u * 4u);
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
        if (!DecodeImageToRgba256(bytes, rgba, wicCtx)) {
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
        constexpr size_t kStaleQueuePressureThreshold = 512;
        constexpr uint64_t kMinStaleTouchDistance = 320;
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
                const auto queuedIt = queuedRequests.find(key);
                if (queuedIt == queuedRequests.end()) {
                    continue;
                }
                if (tileCache.find(key) != tileCache.end()) {
                    queuedRequests.erase(queuedIt);
                    continue;
                }
                const size_t backlog = requestQueue.size();
                const uint64_t touchAge = requestTouchSerial - queuedIt->second.touchSerial;
                const bool staleLowPriority =
                    !queuedIt->second.highPriority &&
                    backlog >= kStaleQueuePressureThreshold &&
                    touchAge > std::max<uint64_t>(kMinStaleTouchDistance, static_cast<uint64_t>(backlog));
                if (staleLowPriority) {
                    queuedRequests.erase(queuedIt);
                    continue;
                }
                queuedRequests.erase(queuedIt);
            }
            (void)FetchAndDecodeTile(key, httpCtx, wicCtx);
        }
        CoUninitialize();
    }

    bool QueueTileRequest(int zoom, int tileX, int tileY, bool highPriority) {
        if (!initialized || zoom < 0 || zoom > 19) {
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
        if (!initialized || zoom < 0 || zoom > 19) {
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

    HudMapTileStats Stats() const {
        HudMapTileStats s{};
        s.diskHits = diskHits.load();
        s.networkHits = networkHits.load();
        s.networkFailures = networkFailures.load();
        s.decodeFailures = decodeFailures.load();
        {
            std::scoped_lock<std::mutex> lock(mutex);
            s.memoryTileCount = tileCache.size();
            s.queuedRequests = queuedRequests.size();
        }
        return s;
    }
};

HudMapTileStreamer::~HudMapTileStreamer() {
    Shutdown();
}

bool HudMapTileStreamer::Initialize(const Config& config, std::string& error) {
    Shutdown();
    m_impl = new Impl();
    if (!m_impl->Initialize(config, error)) {
        delete m_impl;
        m_impl = nullptr;
        return false;
    }
    return true;
}

void HudMapTileStreamer::Shutdown() {
    if (!m_impl) {
        return;
    }
    m_impl->Shutdown();
    delete m_impl;
    m_impl = nullptr;
}

bool HudMapTileStreamer::QueueTileRequest(int zoom, int tileX, int tileY, bool highPriority) {
    if (!m_impl) {
        return false;
    }
    return m_impl->QueueTileRequest(zoom, tileX, tileY, highPriority);
}

bool HudMapTileStreamer::TryGetCachedTileRgba(
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

HudMapTileStats HudMapTileStreamer::GetStats() const {
    if (!m_impl) {
        return {};
    }
    return m_impl->Stats();
}

} // namespace flight::hud
