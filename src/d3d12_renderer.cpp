#include "d3d12_renderer.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <d3dcompiler.h>
#include <imgui.h>
#include <imgui_impl_dx12.h>
#include <imgui_impl_win32.h>
#include <gdal_priv.h>

#include "earth.h"
#include "mesh.h"

namespace flight {
namespace {

using Microsoft::WRL::ComPtr;

constexpr DXGI_FORMAT kBackBufferFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
constexpr DXGI_FORMAT kDepthFormat = DXGI_FORMAT_D32_FLOAT;

UINT64 Align256(UINT64 v) {
    return (v + 255ull) & ~255ull;
}

DirectX::XMFLOAT3 ToFloat3(const Double3& v) {
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

std::string HrMessage(const char* context, HRESULT hr) {
    return std::string(context) + " (hr=0x" + std::to_string(static_cast<unsigned>(hr)) + ")";
}

MeshData GenerateSkyboxCube(float halfExtent) {
    MeshData mesh;
    const float s = halfExtent;
    const DirectX::XMFLOAT3 p[8] = {
        {-s, -s, -s},
        {s, -s, -s},
        {s, s, -s},
        {-s, s, -s},
        {-s, -s, s},
        {s, -s, s},
        {s, s, s},
        {-s, s, s},
    };

    mesh.vertices.resize(8);
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        mesh.vertices[i].position = p[i];
        mesh.vertices[i].normal = {0.0f, 1.0f, 0.0f};
        mesh.vertices[i].uv = {0.0f, 0.0f};
    }

    mesh.indices = {
        4, 5, 6, 4, 6, 7, // +Z
        1, 0, 3, 1, 3, 2, // -Z
        0, 4, 7, 0, 7, 3, // -X
        5, 1, 2, 5, 2, 6, // +X
        3, 7, 6, 3, 6, 2, // +Y
        0, 1, 5, 0, 5, 4, // -Y
    };
    return mesh;
}

bool LoadGeoTiffRgbWithGeoTransform(
    const std::filesystem::path& path,
    uint32_t maxDim,
    std::vector<uint8_t>& outPixelsRgba,
    uint32_t& outWidth,
    uint32_t& outHeight,
    std::array<double, 6>& outGeoTransform,
    bool& outHasGeoTransform,
    std::string& error) {
    error.clear();
    outHasGeoTransform = false;

    static bool gdalRegistered = false;
    if (!gdalRegistered) {
        GDALAllRegister();
        gdalRegistered = true;
    }

    GDALDataset* dataset = static_cast<GDALDataset*>(GDALOpen(path.string().c_str(), GA_ReadOnly));
    if (!dataset) {
        error = "GDALOpen failed for " + path.string();
        return false;
    }

    const int srcW = dataset->GetRasterXSize();
    const int srcH = dataset->GetRasterYSize();
    const int bands = dataset->GetRasterCount();
    if (srcW <= 0 || srcH <= 0 || bands < 3) {
        GDALClose(dataset);
        error = "GeoTIFF missing expected dimensions/bands: " + path.string();
        return false;
    }

    int dstW = srcW;
    int dstH = srcH;
    if (maxDim > 0 && std::max(srcW, srcH) > static_cast<int>(maxDim)) {
        const double scale = static_cast<double>(maxDim) / static_cast<double>(std::max(srcW, srcH));
        dstW = std::max(1, static_cast<int>(std::floor(static_cast<double>(srcW) * scale)));
        dstH = std::max(1, static_cast<int>(std::floor(static_cast<double>(srcH) * scale)));
    }

    double gt[6]{};
    if (dataset->GetGeoTransform(gt) == CE_None) {
        outHasGeoTransform = true;
        // If we downscale the raster on load, scale geotransform terms so lon/lat bounds remain correct.
        const double scaleX = static_cast<double>(srcW) / static_cast<double>(std::max(dstW, 1));
        const double scaleY = static_cast<double>(srcH) / static_cast<double>(std::max(dstH, 1));
        outGeoTransform[0] = gt[0];
        outGeoTransform[1] = gt[1] * scaleX;
        outGeoTransform[2] = gt[2] * scaleY;
        outGeoTransform[3] = gt[3];
        outGeoTransform[4] = gt[4] * scaleX;
        outGeoTransform[5] = gt[5] * scaleY;
    }

    outWidth = static_cast<uint32_t>(dstW);
    outHeight = static_cast<uint32_t>(dstH);
    outPixelsRgba.assign(static_cast<size_t>(dstW) * static_cast<size_t>(dstH) * 4u, 255u);

    // Read RGB bands into RGBA buffer (alpha left as 255).
    int bandMap[3] = {1, 2, 3};
    const CPLErr ioErr = dataset->RasterIO(
        GF_Read,
        0,
        0,
        srcW,
        srcH,
        outPixelsRgba.data(),
        dstW,
        dstH,
        GDT_Byte,
        3,
        bandMap,
        4,
        4 * dstW,
        1);

    GDALClose(dataset);

    if (ioErr != CE_None) {
        error = "GDAL RasterIO failed for " + path.string();
        return false;
    }

    return true;
}

} // namespace

bool D3D12Renderer::Initialize(
    HWND hwnd,
    uint32_t width,
    uint32_t height,
    const std::filesystem::path& shaderDir,
    const std::filesystem::path& assetDir,
    std::string& error) {
    m_hwnd = hwnd;
    m_width = std::max(width, 1u);
    m_height = std::max(height, 1u);
    m_shaderDir = shaderDir;
    m_assetDir = assetDir;

    m_viewport = {0.0f, 0.0f, static_cast<float>(m_width), static_cast<float>(m_height), 0.0f, 1.0f};
    m_scissor = {0, 0, static_cast<LONG>(m_width), static_cast<LONG>(m_height)};

#if defined(_DEBUG)
    {
        ComPtr<ID3D12Debug> debug;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(debug.GetAddressOf())))) {
            debug->EnableDebugLayer();
        }
    }
#endif

    if (!CreateDeviceResources(error)) {
        return false;
    }
    if (!CreateSwapchainResources(error)) {
        return false;
    }
    if (!CreateDepthBuffer(error)) {
        return false;
    }
    if (!CreatePipeline(error)) {
        return false;
    }
    if (!CreateConstantBuffers(error)) {
        return false;
    }
    if (!CreateAtmosphereResources(error)) {
        return false;
    }

    // Upload static resources (earth mesh, fallback plane mesh, skybox mesh, textures).
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
    FrameContext& frame = m_frames[m_frameIndex];

    const HRESULT resetAllocHr = frame.allocator->Reset();
    if (FAILED(resetAllocHr)) {
        error = HrMessage("Command allocator reset failed", resetAllocHr);
        return false;
    }
    const HRESULT resetListHr = m_commandList->Reset(frame.allocator.Get(), nullptr);
    if (FAILED(resetListHr)) {
        error = HrMessage("Command list reset failed", resetListHr);
        return false;
    }

    {
        std::string meshError;
        MeshData earthData = GenerateEarthSphere(static_cast<float>(kEarthRadiusMeters));
        if (!m_earthMesh.Upload(m_device.Get(), m_commandList.Get(), earthData, meshError)) {
            error = "Earth mesh upload failed: " + meshError;
            return false;
        }

        MeshData fallbackPlane = CreatePlaceholderPlane();
        if (!m_planeMesh.Upload(m_device.Get(), m_commandList.Get(), fallbackPlane, meshError)) {
            error = "Fallback plane mesh upload failed: " + meshError;
            return false;
        }

        MeshData skyboxData = GenerateSkyboxCube(1.0f);
        if (!m_skyboxMesh.Upload(m_device.Get(), m_commandList.Get(), skyboxData, meshError)) {
            error = "Skybox mesh upload failed: " + meshError;
            return false;
        }
    }

    if (!CreateLandmaskTexture(m_commandList.Get(), error)) {
        return false;
    }
    if (!CreateSkyboxTexture(m_commandList.Get(), error)) {
        return false;
    }
    if (!CreateEarthAlbedoTexture(m_commandList.Get(), error)) {
        return false;
    }
    if (!CreateWorldStreamingResources(m_commandList.Get(), error)) {
        return false;
    }
    {
        constexpr uint8_t kTransparent[4] = {0, 0, 0, 0};
        if (!CreateSatelliteTextureFromPixels(
                m_commandList.Get(),
                kSatelliteNearSrvIndex,
                m_satelliteLodTextures[0],
                m_satelliteLodUploads[0],
                kTransparent,
                1,
                1,
                error)) {
            return false;
        }
        if (!CreateSatelliteTextureFromPixels(
                m_commandList.Get(),
                kSatelliteMidSrvIndex,
                m_satelliteLodTextures[1],
                m_satelliteLodUploads[1],
                kTransparent,
                1,
                1,
                error)) {
            return false;
        }
        if (!CreateSatelliteTextureFromPixels(
                m_commandList.Get(),
                kSatelliteFarSrvIndex,
                m_satelliteLodTextures[2],
                m_satelliteLodUploads[2],
                kTransparent,
                1,
                1,
                error)) {
            return false;
        }
        if (!CreateSatelliteTextureFromPixels(
                m_commandList.Get(),
                kSatellitePrevNearSrvIndex,
                m_satellitePrevLodTextures[0],
                m_satellitePrevLodUploads[0],
                kTransparent,
                1,
                1,
                error)) {
            return false;
        }
        if (!CreateSatelliteTextureFromPixels(
                m_commandList.Get(),
                kSatellitePrevMidSrvIndex,
                m_satellitePrevLodTextures[1],
                m_satellitePrevLodUploads[1],
                kTransparent,
                1,
                1,
                error)) {
            return false;
        }
        if (!CreateSatelliteTextureFromPixels(
                m_commandList.Get(),
                kSatellitePrevFarSrvIndex,
                m_satellitePrevLodTextures[2],
                m_satellitePrevLodUploads[2],
                kTransparent,
                1,
                1,
                error)) {
            return false;
        }
    }
    {
        constexpr uint8_t kDefaultWhite[4] = {255, 255, 255, 255};
        if (!CreatePlaneTextureFromPixels(m_commandList.Get(), kDefaultWhite, 1, 1, error)) {
            return false;
        }
    }

    const HRESULT closeHr = m_commandList->Close();
    if (FAILED(closeHr)) {
        error = HrMessage("Command list close failed", closeHr);
        return false;
    }

    ID3D12CommandList* lists[] = {m_commandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, lists);
    WaitForGpu();

    m_earthMesh.ReleaseUploadBuffers();
    m_planeMesh.ReleaseUploadBuffers();
    m_skyboxMesh.ReleaseUploadBuffers();
    m_landmaskUpload.Reset();
    m_skyboxUpload.Reset();
    m_earthAlbedoUpload.Reset();
    for (auto& upload : m_satelliteLodUploads) {
        upload.Reset();
    }
    for (auto& upload : m_satellitePrevLodUploads) {
        upload.Reset();
    }
    m_modelAlbedoUpload.Reset();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    if (!ImGui_ImplWin32_Init(m_hwnd)) {
        error = "ImGui Win32 backend initialization failed";
        return false;
    }

    const bool imguiDx12Ok = ImGui_ImplDX12_Init(
        m_device.Get(),
        static_cast<int>(kFrameCount),
        kBackBufferFormat,
        m_srvHeap.Get(),
        CpuSrv(kFontSrvIndex),
        GpuSrv(kFontSrvIndex));

    if (!imguiDx12Ok) {
        error = "ImGui DX12 backend initialization failed";
        return false;
    }
    m_imguiInitialized = true;

    return true;
}

void D3D12Renderer::Shutdown() {
    WaitForGpu();

    if (m_imguiInitialized) {
        ImGui_ImplDX12_Shutdown();
        ImGui_ImplWin32_Shutdown();
        ImGui::DestroyContext();
        m_imguiInitialized = false;
    }

    if (m_fenceEvent != nullptr) {
        CloseHandle(m_fenceEvent);
        m_fenceEvent = nullptr;
    }

    for (UINT i = 0; i < kFrameCount; ++i) {
        if (m_sceneCb[i]) {
            m_sceneCb[i]->Unmap(0, nullptr);
        }
        if (m_objectCb[i]) {
            m_objectCb[i]->Unmap(0, nullptr);
        }
    }

    ReleaseRenderTargets();
    m_depthBuffer.Reset();
    m_commandList.Reset();
    m_uploadCommandList.Reset();
    m_uploadAllocator.Reset();
    m_satelliteUploadCommandList.Reset();
    m_satelliteUploadAllocator.Reset();
    m_worldUploadCommandList.Reset();
    m_worldUploadAllocator.Reset();
    for (auto& frame : m_frames) {
        frame.allocator.Reset();
        frame.fenceValue = 0;
    }

    m_rootSignature.Reset();
    m_computeRootSignature.Reset();
    m_planePso.Reset();
    m_earthPso.Reset();
    m_skyboxPso.Reset();
    m_terrainPso.Reset();
    m_terrainBlendPso.Reset();
    m_transmittanceLutPso.Reset();
    m_skyViewLutPso.Reset();
    m_multiScatteringLutPso.Reset();
    m_aerialPerspectiveLutPso.Reset();

    m_sceneCb = {};
    m_objectCb = {};
    m_landmaskTexture.Reset();
    m_landmaskUpload.Reset();
    m_skyboxTexture.Reset();
    m_skyboxUpload.Reset();
    m_earthAlbedoTexture.Reset();
    m_earthAlbedoUpload.Reset();
    m_worldSatelliteAtlasTexture.Reset();
    m_worldSatellitePageTableTexture.Reset();
    m_worldSatellitePageKeyTexture.Reset();
    m_worldSatellitePageTableCpu.clear();
    m_worldSatellitePageKeyCpu.clear();
    m_worldStreamingResourcesReady = false;
    m_worldStreamingStats = {};
    for (auto& tex : m_satelliteLodTextures) {
        tex.Reset();
    }
    for (auto& tex : m_satellitePrevLodTextures) {
        tex.Reset();
    }
    for (auto& upload : m_satelliteLodUploads) {
        upload.Reset();
    }
    for (auto& upload : m_satellitePrevLodUploads) {
        upload.Reset();
    }
    m_satelliteLodValid = {0.0f, 0.0f, 0.0f};
    m_satellitePrevLodValid = {0.0f, 0.0f, 0.0f};
    m_modelAlbedoTexture.Reset();
    m_modelAlbedoUpload.Reset();
    m_hasModelAlbedoTexture = false;
    m_transmittanceLut.Reset();
    m_skyViewLut.Reset();
    m_multipleScatteringLut.Reset();
    m_aerialPerspectiveLut.Reset();
    m_hasTerrainMesh = false;
    m_hasPrevTerrainMesh = false;
    m_terrainTransitionActive = false;

    m_srvHeap.Reset();
    m_rtvHeap.Reset();
    m_dsvHeap.Reset();

    m_swapChain.Reset();
    m_commandQueue.Reset();
    m_fence.Reset();
    m_device.Reset();
    m_factory.Reset();
    m_wicFactory.Reset();
}

void D3D12Renderer::WaitForGpu() {
    if (!m_commandQueue || !m_fence) {
        return;
    }

    const UINT64 signalValue = ++m_fenceValue;
    m_commandQueue->Signal(m_fence.Get(), signalValue);
    m_fence->SetEventOnCompletion(signalValue, m_fenceEvent);
    WaitForSingleObject(m_fenceEvent, INFINITE);

    for (auto& frame : m_frames) {
        frame.fenceValue = signalValue;
    }
    CleanupDeferredTerrainResources();
}

void D3D12Renderer::Resize(uint32_t width, uint32_t height) {
    if (!m_swapChain || width == 0 || height == 0) {
        return;
    }

    WaitForGpu();
    ReleaseRenderTargets();
    m_depthBuffer.Reset();

    DXGI_SWAP_CHAIN_DESC desc{};
    m_swapChain->GetDesc(&desc);

    if (FAILED(m_swapChain->ResizeBuffers(kFrameCount, width, height, desc.BufferDesc.Format, desc.Flags))) {
        return;
    }

    m_width = width;
    m_height = height;
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

    CreateRenderTargetViews();
    std::string ignore;
    CreateDepthBuffer(ignore);

    m_viewport = {0.0f, 0.0f, static_cast<float>(m_width), static_cast<float>(m_height), 0.0f, 1.0f};
    m_scissor = {0, 0, static_cast<LONG>(m_width), static_cast<LONG>(m_height)};
}

void D3D12Renderer::BeginImGuiFrame() {
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
}

bool D3D12Renderer::SetPlaneMesh(const MeshData& mesh, std::string& error) {
    if (!mesh.IsValid()) {
        error = "SetPlaneMesh called with invalid mesh";
        return false;
    }

    WaitForGpu();

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
    FrameContext& frame = m_frames[m_frameIndex];
    if (FAILED(frame.allocator->Reset())) {
        error = "SetPlaneMesh allocator reset failed";
        return false;
    }
    if (FAILED(m_commandList->Reset(frame.allocator.Get(), nullptr))) {
        error = "SetPlaneMesh command list reset failed";
        return false;
    }

    GpuMesh updated;
    if (!updated.Upload(m_device.Get(), m_commandList.Get(), mesh, error)) {
        return false;
    }

    if (FAILED(m_commandList->Close())) {
        error = "SetPlaneMesh command list close failed";
        return false;
    }

    ID3D12CommandList* lists[] = {m_commandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, lists);
    WaitForGpu();

    updated.ReleaseUploadBuffers();
    m_planeMesh = std::move(updated);
    return true;
}

bool D3D12Renderer::SetPlaneTexture(const uint8_t* rgbaPixels, uint32_t width, uint32_t height, std::string& error) {
    constexpr uint8_t kDefaultWhite[4] = {255, 255, 255, 255};
    const uint8_t* srcPixels = rgbaPixels;
    uint32_t srcWidth = width;
    uint32_t srcHeight = height;
    if (srcPixels == nullptr || srcWidth == 0 || srcHeight == 0) {
        srcPixels = kDefaultWhite;
        srcWidth = 1;
        srcHeight = 1;
    }

    WaitForGpu();

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
    FrameContext& frame = m_frames[m_frameIndex];
    if (FAILED(frame.allocator->Reset())) {
        error = "SetPlaneTexture allocator reset failed";
        return false;
    }
    if (FAILED(m_commandList->Reset(frame.allocator.Get(), nullptr))) {
        error = "SetPlaneTexture command list reset failed";
        return false;
    }

    if (!CreatePlaneTextureFromPixels(m_commandList.Get(), srcPixels, srcWidth, srcHeight, error)) {
        return false;
    }

    if (FAILED(m_commandList->Close())) {
        error = "SetPlaneTexture command list close failed";
        return false;
    }

    ID3D12CommandList* lists[] = {m_commandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, lists);
    WaitForGpu();
    m_modelAlbedoUpload.Reset();

    return true;
}

bool D3D12Renderer::SetSatelliteLodTextures(const std::array<SatelliteLodTexture, 3>& lods, std::string& error) {
    constexpr uint8_t kTransparent[4] = {0, 0, 0, 0};

    if (!m_satelliteUploadAllocator || !m_satelliteUploadCommandList || !m_fence) {
        error = "SetSatelliteLodTextures upload resources are not initialized";
        return false;
    }

    CleanupDeferredTerrainResources();

    if (FAILED(m_satelliteUploadAllocator->Reset())) {
        error = "SetSatelliteLodTextures satellite allocator reset failed";
        return false;
    }
    if (FAILED(m_satelliteUploadCommandList->Reset(m_satelliteUploadAllocator.Get(), nullptr))) {
        error = "SetSatelliteLodTextures satellite command list reset failed";
        return false;
    }

    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 3> newTextures;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 3> newUploads;
    std::array<DirectX::XMFLOAT4, 3> newBounds{};
    std::array<float, 3> newValid{0.0f, 0.0f, 0.0f};

    const UINT64 oldFenceSnapshot = m_fenceValue;
    const bool hadCurrentValid = (m_satelliteLodValid[0] + m_satelliteLodValid[1] + m_satelliteLodValid[2]) > 0.5f;

    for (size_t i = 0; i < lods.size(); ++i) {
        const SatelliteLodTexture& lod = lods[i];
        const uint8_t* pixels = lod.valid ? lod.rgbaPixels : kTransparent;
        const uint32_t width = (lod.valid && lod.width > 0) ? lod.width : 1;
        const uint32_t height = (lod.valid && lod.height > 0) ? lod.height : 1;
        const UINT srvIndex = (i == 0) ? kSatelliteNearSrvIndex : (i == 1 ? kSatelliteMidSrvIndex : kSatelliteFarSrvIndex);
        if (!CreateSatelliteTextureFromPixels(
                m_satelliteUploadCommandList.Get(),
                srvIndex,
                newTextures[i],
                newUploads[i],
                pixels,
                width,
                height,
                error)) {
            return false;
        }
        newBounds[i] = lod.boundsLonLat;
        newValid[i] = lod.valid ? 1.0f : 0.0f;
    }

    if (FAILED(m_satelliteUploadCommandList->Close())) {
        error = "SetSatelliteLodTextures command list close failed";
        return false;
    }

    ID3D12CommandList* lists[] = {m_satelliteUploadCommandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, lists);

    const UINT64 uploadFence = ++m_fenceValue;
    m_commandQueue->Signal(m_fence.Get(), uploadFence);

    for (size_t i = 0; i < m_satelliteLodTextures.size(); ++i) {
        if (m_satellitePrevLodTextures[i]) {
            DeferredResource retiredPrev{};
            retiredPrev.resource = std::move(m_satellitePrevLodTextures[i]);
            retiredPrev.safeFenceValue = oldFenceSnapshot;
            m_retiredResources.push_back(std::move(retiredPrev));
        }
        if (m_satellitePrevLodUploads[i]) {
            DeferredResource retiredPrevUpload{};
            retiredPrevUpload.resource = std::move(m_satellitePrevLodUploads[i]);
            retiredPrevUpload.safeFenceValue = oldFenceSnapshot;
            m_retiredResources.push_back(std::move(retiredPrevUpload));
        }

        m_satellitePrevLodTextures[i] = std::move(m_satelliteLodTextures[i]);
        m_satellitePrevLodUploads[i] = std::move(m_satelliteLodUploads[i]);
        m_satellitePrevLodBounds[i] = m_satelliteLodBounds[i];
        m_satellitePrevLodValid[i] = m_satelliteLodValid[i];

        m_satelliteLodTextures[i] = std::move(newTextures[i]);
        m_satelliteLodUploads[i] = std::move(newUploads[i]);
        m_satelliteLodBounds[i] = newBounds[i];
        m_satelliteLodValid[i] = newValid[i];
    }
    if (m_satellitePrevLodTextures[0]) {
        CreateSatelliteSrvForResource(m_satellitePrevLodTextures[0].Get(), kSatellitePrevNearSrvIndex);
    }
    if (m_satellitePrevLodTextures[1]) {
        CreateSatelliteSrvForResource(m_satellitePrevLodTextures[1].Get(), kSatellitePrevMidSrvIndex);
    }
    if (m_satellitePrevLodTextures[2]) {
        CreateSatelliteSrvForResource(m_satellitePrevLodTextures[2].Get(), kSatellitePrevFarSrvIndex);
    }
    m_satelliteUploadFenceValue = uploadFence;

    const bool hasNewValid = (newValid[0] + newValid[1] + newValid[2]) > 0.5f;
    auto sameBounds = [](const DirectX::XMFLOAT4& a, const DirectX::XMFLOAT4& b) {
        return std::abs(a.x - b.x) < 1e-5f &&
            std::abs(a.y - b.y) < 1e-5f &&
            std::abs(a.z - b.z) < 1e-5f &&
            std::abs(a.w - b.w) < 1e-5f;
    };
    bool mappingChanged = false;
    for (size_t i = 0; i < m_satelliteLodBounds.size(); ++i) {
        if (!sameBounds(newBounds[i], m_satellitePrevLodBounds[i]) || std::abs(newValid[i] - m_satellitePrevLodValid[i]) > 1e-3f) {
            mappingChanged = true;
            break;
        }
    }

    if (hadCurrentValid && hasNewValid && mappingChanged) {
        m_satelliteTransitionStart = std::chrono::steady_clock::now();
        m_satelliteTransitionT = 0.0f;
        m_satelliteTransitionActive = true;
    } else {
        m_satelliteTransitionT = 1.0f;
        m_satelliteTransitionActive = false;
    }

    return true;
}

bool D3D12Renderer::SetTerrainMesh(
    const MeshData& mesh,
    const Double3& anchorEcef,
    const DirectX::XMFLOAT4& renderParams,
    std::string& error) {
    if (!mesh.IsValid()) {
        error = "SetTerrainMesh called with invalid mesh";
        return false;
    }

    if (!m_uploadAllocator || !m_uploadCommandList || !m_fence) {
        error = "SetTerrainMesh upload resources are not initialized";
        return false;
    }

    CleanupDeferredTerrainResources();

    if (FAILED(m_uploadAllocator->Reset())) {
        error = "SetTerrainMesh upload allocator reset failed";
        return false;
    }
    if (FAILED(m_uploadCommandList->Reset(m_uploadAllocator.Get(), nullptr))) {
        error = "SetTerrainMesh upload command list reset failed";
        return false;
    }

    GpuMesh updated;
    if (!updated.Upload(m_device.Get(), m_uploadCommandList.Get(), mesh, error)) {
        return false;
    }

    if (FAILED(m_uploadCommandList->Close())) {
        error = "SetTerrainMesh upload command list close failed";
        return false;
    }

    ID3D12CommandList* lists[] = {m_uploadCommandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, lists);
    const UINT64 oldFenceSnapshot = m_fenceValue;
    const UINT64 uploadFence = ++m_fenceValue;
    m_commandQueue->Signal(m_fence.Get(), uploadFence);

    if (m_hasPrevTerrainMesh && m_prevTerrainMesh.IsValid()) {
        DeferredTerrainMesh retiredPrev{};
        retiredPrev.mesh = std::move(m_prevTerrainMesh);
        retiredPrev.safeFenceValue = std::max(oldFenceSnapshot, m_terrainUploadFenceValue);
        m_retiredTerrainMeshes.push_back(std::move(retiredPrev));
        m_hasPrevTerrainMesh = false;
    }

    if (m_hasTerrainMesh && m_terrainMesh.IsValid()) {
        m_prevTerrainMesh = std::move(m_terrainMesh);
        m_prevTerrainAnchorEcef = m_terrainAnchorEcef;
        m_prevTerrainRenderParams = m_terrainRenderParams;
        m_hasPrevTerrainMesh = true;
    }

    m_terrainMesh = std::move(updated);
    m_terrainUploadFenceValue = uploadFence;
    m_terrainAnchorEcef = anchorEcef;
    m_terrainRenderParams = renderParams;
    m_hasTerrainMesh = true;
    if (m_hasPrevTerrainMesh) {
        m_terrainTransitionStart = std::chrono::steady_clock::now();
        m_terrainTransitionT = 0.0f;
        m_terrainTransitionActive = true;
    } else {
        m_terrainTransitionT = 1.0f;
        m_terrainTransitionActive = false;
    }
    return true;
}

void D3D12Renderer::SetTerrainVisualSettings(const TerrainVisualSettings& settings) {
    TerrainVisualSettings clamped = settings;
    clamped.colorHeightMaxMeters = std::clamp(clamped.colorHeightMaxMeters, 1000.0f, 20000.0f);
    clamped.lodTransitionWidthMeters = std::clamp(clamped.lodTransitionWidthMeters, 250.0f, 50000.0f);
    clamped.midRingMultiplier = std::clamp(clamped.midRingMultiplier, 1.1f, 5.0f);
    clamped.farRingMultiplier = std::clamp(clamped.farRingMultiplier, 1.1f, 5.0f);
    clamped.hazeStrength = std::clamp(clamped.hazeStrength, 0.0f, 2.0f);
    clamped.hazeAltitudeRangeMeters = std::clamp(clamped.hazeAltitudeRangeMeters, 1000.0f, 80000.0f);
    clamped.colorContrast = std::clamp(clamped.colorContrast, 0.2f, 3.0f);
    clamped.slopeShadingStrength = std::clamp(clamped.slopeShadingStrength, 0.0f, 2.0f);
    clamped.specularStrength = std::clamp(clamped.specularStrength, 0.0f, 1.0f);
    clamped.lodSeamBlendStrength = std::clamp(clamped.lodSeamBlendStrength, 0.0f, 2.0f);
    clamped.satelliteBlend = std::clamp(clamped.satelliteBlend, 0.0f, 1.0f);
    m_terrainVisualSettings = clamped;
}

void D3D12Renderer::SetAerialPerspectiveDepthMeters(float depthMeters) {
    m_aerialPerspectiveDepthMeters = std::clamp(depthMeters, 5000.0f, 1000000.0f);
}

void D3D12Renderer::SetSunDirection(const Double3& dir) {
    const Double3 normalized = Normalize(dir);
    if (Length(normalized) < 1e-8) {
        return;
    }
    m_sunDirection = normalized;
}

void D3D12Renderer::SetCameraZoomRangeMeters(float minDistance, float maxDistance) {
    const float clampedMin = std::clamp(minDistance, 2.0f, 10000.0f);
    const float clampedMax = std::clamp(maxDistance, clampedMin + 1.0f, 50000.0f);
    m_cameraMinFollowDistanceMeters = clampedMin;
    m_cameraMaxFollowDistanceMeters = clampedMax;
    m_cameraFollowDistanceMeters =
        std::clamp(m_cameraFollowDistanceMeters, m_cameraMinFollowDistanceMeters, m_cameraMaxFollowDistanceMeters);
}

void D3D12Renderer::AddCameraZoomSteps(float wheelSteps) {
    if (std::abs(wheelSteps) < 1e-5f) {
        return;
    }
    constexpr float kZoomMetersPerStep = 30.0f;
    // Wheel up should zoom in (reduce follow distance).
    m_cameraFollowDistanceMeters -= wheelSteps * kZoomMetersPerStep;
    m_cameraFollowDistanceMeters =
        std::clamp(m_cameraFollowDistanceMeters, m_cameraMinFollowDistanceMeters, m_cameraMaxFollowDistanceMeters);
}

void D3D12Renderer::Render(const FlightSim& sim, ImDrawData* imguiDrawData) {
    if (!m_swapChain) {
        return;
    }

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
    WaitForFrame(m_frameIndex);
    CleanupDeferredTerrainResources();

    FrameContext& frame = m_frames[m_frameIndex];
    frame.allocator->Reset();
    m_commandList->Reset(frame.allocator.Get(), nullptr);

    D3D12_RESOURCE_BARRIER toRender{};
    toRender.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    toRender.Transition.pResource = m_renderTargets[m_frameIndex].Get();
    toRender.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
    toRender.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
    toRender.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    m_commandList->ResourceBarrier(1, &toRender);

    const D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = {
        m_rtvHeap->GetCPUDescriptorHandleForHeapStart().ptr + static_cast<SIZE_T>(m_frameIndex) * m_rtvDescriptorSize};
    const D3D12_CPU_DESCRIPTOR_HANDLE dsvHandle = m_dsvHeap->GetCPUDescriptorHandleForHeapStart();

    constexpr float clearColor[4] = {0.02f, 0.04f, 0.09f, 1.0f};
    m_commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);
    m_commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    m_commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

    m_commandList->RSSetViewports(1, &m_viewport);
    m_commandList->RSSetScissorRects(1, &m_scissor);

    ID3D12DescriptorHeap* heaps[] = {m_srvHeap.Get()};
    m_commandList->SetDescriptorHeaps(1, heaps);

    const Double3 planeEcef = sim.PlaneEcef();
    const Double3 forwardEcef = Normalize(sim.ForwardEcef());
    const Double3 upEcef = Normalize(sim.UpEcef());

    // Third-person chase camera with wheel-controlled follow distance.
    const double kCameraFollowDistance = static_cast<double>(m_cameraFollowDistanceMeters);
    const double kCameraHeight = std::clamp(kCameraFollowDistance * 0.32, 18.0, 420.0);
    const double kCameraLookAhead = std::clamp(kCameraFollowDistance * 0.35, 28.0, 540.0);
    constexpr float kCameraFovDegrees = 50.0f;

    const Double3 cameraEcef = planeEcef - forwardEcef * kCameraFollowDistance + upEcef * kCameraHeight;
    const Double3 anchor = cameraEcef;

    const Double3 planeLocalD = planeEcef - anchor;
    const Double3 earthCenterLocalD = Double3{-anchor.x, -anchor.y, -anchor.z};
    const Double3 cameraTarget = planeLocalD + forwardEcef * kCameraLookAhead;

    const DirectX::XMVECTOR eye = DirectX::XMVectorZero();
    const DirectX::XMVECTOR at = DirectX::XMVectorSet(
        static_cast<float>(cameraTarget.x),
        static_cast<float>(cameraTarget.y),
        static_cast<float>(cameraTarget.z),
        1.0f);
    const DirectX::XMVECTOR up = DirectX::XMVectorSet(
        static_cast<float>(upEcef.x),
        static_cast<float>(upEcef.y),
        static_cast<float>(upEcef.z),
        0.0f);

    const DirectX::XMMATRIX view = DirectX::XMMatrixLookAtLH(eye, at, up);
    const float aspect = static_cast<float>(m_width) / static_cast<float>(std::max(m_height, 1u));
    const DirectX::XMMATRIX proj = DirectX::XMMatrixPerspectiveFovLH(
        DirectX::XMConvertToRadians(kCameraFovDegrees),
        aspect,
        10.0f,
        50000000.0f);
    const DirectX::XMMATRIX viewProj = view * proj;
    const DirectX::XMMATRIX invViewProj = DirectX::XMMatrixInverse(nullptr, viewProj);
    const Double3 cameraUp = Normalize(Double3{-earthCenterLocalD.x, -earthCenterLocalD.y, -earthCenterLocalD.z});
    const float topRadius = static_cast<float>(kEarthRadiusMeters + static_cast<double>(m_atmosphereSettings.atmosphereHeightMeters));

    SceneConstants sceneCb{};
    DirectX::XMStoreFloat4x4(&sceneCb.viewProj, DirectX::XMMatrixTranspose(viewProj));
    DirectX::XMStoreFloat4x4(&sceneCb.invViewProj, DirectX::XMMatrixTranspose(invViewProj));
    sceneCb.earthCenterRadius = {
        static_cast<float>(earthCenterLocalD.x),
        static_cast<float>(earthCenterLocalD.y),
        static_cast<float>(earthCenterLocalD.z),
        static_cast<float>(kEarthRadiusMeters),
    };
    const Double3 sunDir = Normalize(m_sunDirection);
    sceneCb.sunDirIntensity = {
        static_cast<float>(sunDir.x),
        static_cast<float>(sunDir.y),
        static_cast<float>(sunDir.z),
        m_atmosphereSettings.sunIlluminance,
    };
    sceneCb.cameraPosTopRadius = {
        0.0f,
        0.0f,
        0.0f,
        topRadius,
    };
    sceneCb.cameraUpAndTime = {
        static_cast<float>(cameraUp.x),
        static_cast<float>(cameraUp.y),
        static_cast<float>(cameraUp.z),
        static_cast<float>(GetTickCount64() * 0.001),
    };
    sceneCb.viewportAndAerialDepth = {
        static_cast<float>(m_width),
        static_cast<float>(m_height),
        m_aerialPerspectiveDepthMeters,
        m_atmosphereEnabled ? 1.0f : 0.0f,
    };
    sceneCb.atmosphereFlags = {
        m_multipleScatteringEnabled ? 1.0f : 0.0f,
        m_atmosphereSettings.miePhaseG,
        m_atmosphereExposure,
        0.0f,
    };
    sceneCb.rayleighScatteringAndScale = {
        m_atmosphereSettings.rayleighScattering.x,
        m_atmosphereSettings.rayleighScattering.y,
        m_atmosphereSettings.rayleighScattering.z,
        m_atmosphereSettings.rayleighScaleHeightMeters,
    };
    sceneCb.rayleighAbsorptionPad = {
        m_atmosphereSettings.rayleighAbsorption.x,
        m_atmosphereSettings.rayleighAbsorption.y,
        m_atmosphereSettings.rayleighAbsorption.z,
        0.0f,
    };
    sceneCb.mieScatteringAndScale = {
        m_atmosphereSettings.mieScattering.x,
        m_atmosphereSettings.mieScattering.y,
        m_atmosphereSettings.mieScattering.z,
        m_atmosphereSettings.mieScaleHeightMeters,
    };
    sceneCb.mieAbsorptionAndG = {
        m_atmosphereSettings.mieAbsorption.x,
        m_atmosphereSettings.mieAbsorption.y,
        m_atmosphereSettings.mieAbsorption.z,
        m_atmosphereSettings.miePhaseG,
    };
    sceneCb.ozoneAbsorptionAndCenter = {
        m_atmosphereSettings.ozoneAbsorption.x,
        m_atmosphereSettings.ozoneAbsorption.y,
        m_atmosphereSettings.ozoneAbsorption.z,
        m_atmosphereSettings.ozoneCenterHeightMeters,
    };
    sceneCb.ozoneHalfWidthAtmosphereHeightSunRadius = {
        m_atmosphereSettings.ozoneHalfWidthMeters,
        m_atmosphereSettings.atmosphereHeightMeters,
        m_atmosphereSettings.sunAngularRadiusRad,
        0.0f,
    };
    std::memcpy(m_sceneCbMapped[m_frameIndex], &sceneCb, sizeof(sceneCb));

    if (m_satelliteTransitionActive) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_satelliteTransitionStart).count();
        const double dur = std::max(0.05, static_cast<double>(m_satelliteTransitionDurationSeconds));
        m_satelliteTransitionT = static_cast<float>(std::clamp(elapsed / dur, 0.0, 1.0));
        if (m_satelliteTransitionT >= 1.0f) {
            m_satelliteTransitionT = 1.0f;
            m_satelliteTransitionActive = false;
        }
    } else {
        m_satelliteTransitionT = 1.0f;
    }

    const D3D12_GPU_VIRTUAL_ADDRESS sceneCbGpu = m_sceneCb[m_frameIndex]->GetGPUVirtualAddress();
    const D3D12_GPU_VIRTUAL_ADDRESS objectCbGpuBase = m_objectCb[m_frameIndex]->GetGPUVirtualAddress();

    DispatchAtmosphereLuts(m_commandList.Get(), sceneCb);

    m_commandList->SetGraphicsRootSignature(m_rootSignature.Get());
    m_commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    m_commandList->SetGraphicsRootConstantBufferView(0, sceneCbGpu);
    m_commandList->SetGraphicsRootDescriptorTable(2, GpuSrv(kSrvTableStartIndex));

    // Draw skybox first with depth disabled.
    {
        ObjectConstants obj{};
        const DirectX::XMMATRIX model = DirectX::XMMatrixScaling(30000000.0f, 30000000.0f, 30000000.0f);
        DirectX::XMStoreFloat4x4(&obj.model, DirectX::XMMatrixTranspose(model));
        obj.colorAndFlags = {1.0f, 1.0f, 1.0f, 0.0f};
        obj.tuning0 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning1 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning2 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning3 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning4 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning5 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning6 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning7 = {0.0f, 0.0f, 0.0f, 0.0f};

        std::memcpy(m_objectCbMapped[m_frameIndex] + (m_objectCbStride * 0), &obj, sizeof(obj));
        m_commandList->SetGraphicsRootConstantBufferView(1, objectCbGpuBase + m_objectCbStride * 0);
        m_commandList->SetPipelineState(m_skyboxPso.Get());
        m_skyboxMesh.Draw(m_commandList.Get());
    }

    // Draw Earth sphere (optional legacy ground/ocean backdrop).
    if (m_renderOldEarthSphere) {
        ObjectConstants obj{};
        const DirectX::XMMATRIX model = DirectX::XMMatrixTranslation(
            static_cast<float>(earthCenterLocalD.x),
            static_cast<float>(earthCenterLocalD.y),
            static_cast<float>(earthCenterLocalD.z));
        DirectX::XMStoreFloat4x4(&obj.model, DirectX::XMMatrixTranspose(model));
        obj.colorAndFlags = {0.25f, 0.60f, 0.22f, m_hasLandmaskTexture ? 1.0f : 0.0f};
        obj.tuning0 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning1 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning2 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning3 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning4 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning5 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning6 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning7 = {0.0f, 0.0f, 0.0f, 0.0f};

        std::memcpy(m_objectCbMapped[m_frameIndex] + (m_objectCbStride * 1), &obj, sizeof(obj));
        m_commandList->SetGraphicsRootConstantBufferView(1, objectCbGpuBase + m_objectCbStride * 1);
        m_commandList->SetPipelineState(m_earthPso.Get());
        m_earthMesh.Draw(m_commandList.Get());
    }

    // Draw primary terrain patch from ETOPO samples (with transition crossfade on patch rebuilds).
    if (m_terrainTransitionActive) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_terrainTransitionStart).count();
        const double dur = std::max(0.05, static_cast<double>(m_terrainTransitionDurationSeconds));
        m_terrainTransitionT = static_cast<float>(std::clamp(elapsed / dur, 0.0, 1.0));
        if (m_terrainTransitionT >= 1.0f) {
            m_terrainTransitionT = 1.0f;
            m_terrainTransitionActive = false;
            if (m_hasPrevTerrainMesh && m_prevTerrainMesh.IsValid()) {
                DeferredTerrainMesh retiredPrev{};
                retiredPrev.mesh = std::move(m_prevTerrainMesh);
                retiredPrev.safeFenceValue = std::max(m_fenceValue, m_terrainUploadFenceValue);
                m_retiredTerrainMeshes.push_back(std::move(retiredPrev));
                m_hasPrevTerrainMesh = false;
            }
        }
    } else {
        m_terrainTransitionT = 1.0f;
    }

    auto drawTerrainLayer = [&](const GpuMesh& mesh, const Double3& meshAnchor, const DirectX::XMFLOAT4& meshParams, float layerAlpha, bool useBlendPso) {
        if (!mesh.IsValid() || layerAlpha <= 1e-3f) {
            return;
        }
        const Double3 terrainOffsetD = meshAnchor - anchor;
        ObjectConstants obj{};
        const DirectX::XMMATRIX model = DirectX::XMMatrixTranslation(
            static_cast<float>(terrainOffsetD.x),
            static_cast<float>(terrainOffsetD.y),
            static_cast<float>(terrainOffsetD.z));
        DirectX::XMStoreFloat4x4(&obj.model, DirectX::XMMatrixTranspose(model));
        DirectX::XMFLOAT4 params = meshParams;
        params.y = m_terrainVisualSettings.colorHeightMaxMeters;
        params.z = m_terrainVisualSettings.lodTransitionWidthMeters;
        obj.colorAndFlags = params;
        obj.tuning0 = {
            m_terrainVisualSettings.midRingMultiplier,
            m_terrainVisualSettings.farRingMultiplier,
            m_terrainVisualSettings.hazeStrength,
            m_terrainVisualSettings.hazeAltitudeRangeMeters,
        };
        obj.tuning1 = {
            m_terrainVisualSettings.colorContrast,
            m_terrainVisualSettings.slopeShadingStrength,
            m_terrainVisualSettings.specularStrength,
            m_terrainVisualSettings.lodSeamBlendStrength,
        };
        obj.tuning2 = {
            m_terrainVisualSettings.satelliteEnabled ? 1.0f : 0.0f,
            m_terrainVisualSettings.satelliteBlend,
            m_earthAlbedoWrapLon,
            m_satelliteLodValid[0] + m_satelliteLodValid[1] + m_satelliteLodValid[2],
        };
        obj.tuning3 = m_earthAlbedoBoundsLonLat;
        obj.tuning4 = m_satelliteLodBounds[0];
        obj.tuning5 = m_satelliteLodBounds[1];
        obj.tuning6 = m_satelliteLodBounds[2];
        obj.tuning7 = {m_satelliteLodValid[0], m_satelliteLodValid[1], m_satelliteLodValid[2], 0.0f};
        obj.tuning8 = m_satellitePrevLodBounds[0];
        obj.tuning9 = m_satellitePrevLodBounds[1];
        obj.tuning10 = m_satellitePrevLodBounds[2];
        obj.tuning11 = {m_satellitePrevLodValid[0], m_satellitePrevLodValid[1], m_satellitePrevLodValid[2], m_satelliteTransitionT};
        const bool worldSamplingEnabled = m_worldLockedSatelliteEnabled && m_worldStreamingResourcesReady;
        obj.tuning12 = {layerAlpha, worldSamplingEnabled ? 1.0f : 0.0f, 1.0f, 0.0f};
        obj.tuning13 = {
            static_cast<float>(kWorldSatelliteAtlasPagesX),
            static_cast<float>(kWorldSatelliteAtlasPagesY),
            static_cast<float>(kWorldSatellitePageTableWidth),
            static_cast<float>(kWorldSatellitePageTableHeight),
        };
        obj.tuning14 = {
            static_cast<float>(std::clamp(m_worldSamplingZooms[0], 0, 22)),
            static_cast<float>(std::clamp(m_worldSamplingZooms[1], 0, 22)),
            static_cast<float>(std::clamp(m_worldSamplingZooms[2], 0, 22)),
            2.0f,
        };

        std::memcpy(m_objectCbMapped[m_frameIndex] + (m_objectCbStride * 2), &obj, sizeof(obj));
        m_commandList->SetGraphicsRootConstantBufferView(1, objectCbGpuBase + m_objectCbStride * 2);
        m_commandList->SetPipelineState(useBlendPso ? m_terrainBlendPso.Get() : m_terrainPso.Get());
        mesh.Draw(m_commandList.Get());
    };

    if (m_hasTerrainMesh && m_terrainMesh.IsValid()) {
        if (m_terrainTransitionActive && m_hasPrevTerrainMesh && m_prevTerrainMesh.IsValid()) {
            // Keep depth stable by drawing current terrain opaque, then fading previous terrain out on top.
            drawTerrainLayer(m_terrainMesh, m_terrainAnchorEcef, m_terrainRenderParams, 1.0f, false);
            drawTerrainLayer(m_prevTerrainMesh, m_prevTerrainAnchorEcef, m_prevTerrainRenderParams, 1.0f - m_terrainTransitionT, true);
        } else {
            drawTerrainLayer(m_terrainMesh, m_terrainAnchorEcef, m_terrainRenderParams, 1.0f, false);
        }
    }

    // Draw plane.
    {
        Double3 rightEcef = Normalize(Cross(upEcef, forwardEcef));
        if (Length(rightEcef) < 1e-6) {
            rightEcef = {1.0, 0.0, 0.0};
        }
        Double3 upOrtho = Normalize(Cross(forwardEcef, rightEcef));

        const double c = std::cos(sim.RollRad());
        const double s = std::sin(sim.RollRad());
        const Double3 rightRolled = Normalize(rightEcef * c + upOrtho * s);
        const Double3 upRolled = Normalize(Cross(forwardEcef, rightRolled));

        const DirectX::XMFLOAT3 pos = ToFloat3(planeLocalD);

        const DirectX::XMMATRIX model(
            static_cast<float>(rightRolled.x), static_cast<float>(rightRolled.y), static_cast<float>(rightRolled.z), 0.0f,
            static_cast<float>(upRolled.x), static_cast<float>(upRolled.y), static_cast<float>(upRolled.z), 0.0f,
            static_cast<float>(forwardEcef.x), static_cast<float>(forwardEcef.y), static_cast<float>(forwardEcef.z), 0.0f,
            pos.x, pos.y, pos.z, 1.0f);

        ObjectConstants obj{};
        DirectX::XMStoreFloat4x4(&obj.model, DirectX::XMMatrixTranspose(model));
        obj.colorAndFlags = {1.0f, 1.0f, 1.0f, m_hasModelAlbedoTexture ? 1.0f : 0.0f};
        obj.tuning0 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning1 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning2 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning3 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning4 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning5 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning6 = {0.0f, 0.0f, 0.0f, 0.0f};
        obj.tuning7 = {0.0f, 0.0f, 0.0f, 0.0f};

        std::memcpy(m_objectCbMapped[m_frameIndex] + (m_objectCbStride * 3), &obj, sizeof(obj));
        m_commandList->SetGraphicsRootConstantBufferView(1, objectCbGpuBase + m_objectCbStride * 3);
        m_commandList->SetPipelineState(m_planePso.Get());
        m_planeMesh.Draw(m_commandList.Get());
    }

    if (imguiDrawData != nullptr) {
        ImGui_ImplDX12_RenderDrawData(imguiDrawData, m_commandList.Get());
    }

    D3D12_RESOURCE_BARRIER toPresent{};
    toPresent.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    toPresent.Transition.pResource = m_renderTargets[m_frameIndex].Get();
    toPresent.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
    toPresent.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
    toPresent.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    m_commandList->ResourceBarrier(1, &toPresent);

    m_commandList->Close();

    ID3D12CommandList* lists[] = {m_commandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, lists);

    m_swapChain->Present(1, 0);

    const UINT64 signalValue = ++m_fenceValue;
    m_commandQueue->Signal(m_fence.Get(), signalValue);
    m_frames[m_frameIndex].fenceValue = signalValue;
}

bool D3D12Renderer::CreateDeviceResources(std::string& error) {
    UINT dxgiFlags = 0;
#if defined(_DEBUG)
    dxgiFlags |= DXGI_CREATE_FACTORY_DEBUG;
#endif

    HRESULT hr = CreateDXGIFactory2(dxgiFlags, IID_PPV_ARGS(m_factory.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateDXGIFactory2 failed", hr);
        return false;
    }

    hr = D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(m_device.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("D3D12CreateDevice failed", hr);
        return false;
    }

    D3D12_COMMAND_QUEUE_DESC queueDesc{};
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    hr = m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(m_commandQueue.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandQueue failed", hr);
        return false;
    }

    D3D12_DESCRIPTOR_HEAP_DESC rtvDesc{};
    rtvDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    rtvDesc.NumDescriptors = kFrameCount;
    hr = m_device->CreateDescriptorHeap(&rtvDesc, IID_PPV_ARGS(m_rtvHeap.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateDescriptorHeap(RTV) failed", hr);
        return false;
    }

    D3D12_DESCRIPTOR_HEAP_DESC dsvDesc{};
    dsvDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
    dsvDesc.NumDescriptors = 1;
    hr = m_device->CreateDescriptorHeap(&dsvDesc, IID_PPV_ARGS(m_dsvHeap.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateDescriptorHeap(DSV) failed", hr);
        return false;
    }

    D3D12_DESCRIPTOR_HEAP_DESC srvDesc{};
    srvDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    srvDesc.NumDescriptors = 32;
    srvDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    hr = m_device->CreateDescriptorHeap(&srvDesc, IID_PPV_ARGS(m_srvHeap.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateDescriptorHeap(SRV) failed", hr);
        return false;
    }

    m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    m_srvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    for (UINT i = 0; i < kFrameCount; ++i) {
        hr = m_device->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_DIRECT,
            IID_PPV_ARGS(m_frames[i].allocator.ReleaseAndGetAddressOf()));
        if (FAILED(hr)) {
            error = HrMessage("CreateCommandAllocator failed", hr);
            return false;
        }
    }

    hr = m_device->CreateCommandList(
        0,
        D3D12_COMMAND_LIST_TYPE_DIRECT,
        m_frames[0].allocator.Get(),
        nullptr,
        IID_PPV_ARGS(m_commandList.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandList failed", hr);
        return false;
    }
    m_commandList->Close();

    hr = m_device->CreateCommandAllocator(
        D3D12_COMMAND_LIST_TYPE_DIRECT,
        IID_PPV_ARGS(m_uploadAllocator.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandAllocator(upload) failed", hr);
        return false;
    }

    hr = m_device->CreateCommandList(
        0,
        D3D12_COMMAND_LIST_TYPE_DIRECT,
        m_uploadAllocator.Get(),
        nullptr,
        IID_PPV_ARGS(m_uploadCommandList.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandList(upload) failed", hr);
        return false;
    }
    m_uploadCommandList->Close();

    hr = m_device->CreateCommandAllocator(
        D3D12_COMMAND_LIST_TYPE_DIRECT,
        IID_PPV_ARGS(m_satelliteUploadAllocator.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandAllocator(satellite upload) failed", hr);
        return false;
    }

    hr = m_device->CreateCommandList(
        0,
        D3D12_COMMAND_LIST_TYPE_DIRECT,
        m_satelliteUploadAllocator.Get(),
        nullptr,
        IID_PPV_ARGS(m_satelliteUploadCommandList.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandList(satellite upload) failed", hr);
        return false;
    }
    m_satelliteUploadCommandList->Close();

    hr = m_device->CreateCommandAllocator(
        D3D12_COMMAND_LIST_TYPE_DIRECT,
        IID_PPV_ARGS(m_worldUploadAllocator.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandAllocator(world upload) failed", hr);
        return false;
    }

    hr = m_device->CreateCommandList(
        0,
        D3D12_COMMAND_LIST_TYPE_DIRECT,
        m_worldUploadAllocator.Get(),
        nullptr,
        IID_PPV_ARGS(m_worldUploadCommandList.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommandList(world upload) failed", hr);
        return false;
    }
    m_worldUploadCommandList->Close();

    hr = m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(m_fence.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateFence failed", hr);
        return false;
    }

    m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (m_fenceEvent == nullptr) {
        error = "CreateEvent for fence failed";
        return false;
    }

    hr = CoCreateInstance(
        CLSID_WICImagingFactory,
        nullptr,
        CLSCTX_INPROC_SERVER,
        IID_PPV_ARGS(m_wicFactory.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CoCreateInstance(WICImagingFactory) failed", hr);
        return false;
    }

    return true;
}

bool D3D12Renderer::CreateSwapchainResources(std::string& error) {
    DXGI_SWAP_CHAIN_DESC1 desc{};
    desc.Width = m_width;
    desc.Height = m_height;
    desc.Format = kBackBufferFormat;
    desc.SampleDesc.Count = 1;
    desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    desc.BufferCount = kFrameCount;
    desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;

    ComPtr<IDXGISwapChain1> swapChain1;
    HRESULT hr = m_factory->CreateSwapChainForHwnd(
        m_commandQueue.Get(),
        m_hwnd,
        &desc,
        nullptr,
        nullptr,
        swapChain1.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        error = HrMessage("CreateSwapChainForHwnd failed", hr);
        return false;
    }

    hr = swapChain1.As(&m_swapChain);
    if (FAILED(hr)) {
        error = HrMessage("Query IDXGISwapChain3 failed", hr);
        return false;
    }

    m_factory->MakeWindowAssociation(m_hwnd, DXGI_MWA_NO_ALT_ENTER);

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
    CreateRenderTargetViews();
    return true;
}

bool D3D12Renderer::CreateDepthBuffer(std::string& error) {
    D3D12_HEAP_PROPERTIES heapProps{};
    heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;

    D3D12_RESOURCE_DESC desc{};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    desc.Width = m_width;
    desc.Height = m_height;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.Format = kDepthFormat;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

    D3D12_CLEAR_VALUE clearValue{};
    clearValue.Format = kDepthFormat;
    clearValue.DepthStencil.Depth = 1.0f;

    const HRESULT hr = m_device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &desc,
        D3D12_RESOURCE_STATE_DEPTH_WRITE,
        &clearValue,
        IID_PPV_ARGS(m_depthBuffer.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(depth) failed", hr);
        return false;
    }

    D3D12_DEPTH_STENCIL_VIEW_DESC dsv{};
    dsv.Format = kDepthFormat;
    dsv.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    dsv.Flags = D3D12_DSV_FLAG_NONE;
    m_device->CreateDepthStencilView(m_depthBuffer.Get(), &dsv, m_dsvHeap->GetCPUDescriptorHandleForHeapStart());

    return true;
}

bool D3D12Renderer::CompileShader(
    const std::filesystem::path& path,
    const char* entry,
    const char* target,
    ComPtr<ID3DBlob>& outBlob,
    std::string& error) {
    UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined(_DEBUG)
    flags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

    ComPtr<ID3DBlob> errors;
    const std::wstring ws = path.wstring();
    const HRESULT hr = D3DCompileFromFile(
        ws.c_str(),
        nullptr,
        D3D_COMPILE_STANDARD_FILE_INCLUDE,
        entry,
        target,
        flags,
        0,
        outBlob.ReleaseAndGetAddressOf(),
        errors.GetAddressOf());

    if (FAILED(hr)) {
        std::string compilerErrors;
        if (errors) {
            compilerErrors.assign(
                static_cast<const char*>(errors->GetBufferPointer()),
                errors->GetBufferSize());
        }
        error = "Shader compile failed for " + path.string() + ": " + compilerErrors;
        return false;
    }

    return true;
}

bool D3D12Renderer::CreatePipeline(std::string& error) {
    D3D12_DESCRIPTOR_RANGE graphicsSrvRanges[2]{};
    graphicsSrvRanges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    graphicsSrvRanges[0].NumDescriptors = 16; // t0..t15
    graphicsSrvRanges[0].BaseShaderRegister = 0;
    graphicsSrvRanges[0].RegisterSpace = 0;
    graphicsSrvRanges[0].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
    graphicsSrvRanges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    graphicsSrvRanges[1].NumDescriptors = 3; // t19..t21
    graphicsSrvRanges[1].BaseShaderRegister = 19;
    graphicsSrvRanges[1].RegisterSpace = 0;
    graphicsSrvRanges[1].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER graphicsParams[3]{};

    graphicsParams[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    graphicsParams[0].Descriptor.ShaderRegister = 0;
    graphicsParams[0].Descriptor.RegisterSpace = 0;
    graphicsParams[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    graphicsParams[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    graphicsParams[1].Descriptor.ShaderRegister = 1;
    graphicsParams[1].Descriptor.RegisterSpace = 0;
    graphicsParams[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    graphicsParams[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    graphicsParams[2].DescriptorTable.NumDescriptorRanges = static_cast<UINT>(std::size(graphicsSrvRanges));
    graphicsParams[2].DescriptorTable.pDescriptorRanges = graphicsSrvRanges;
    graphicsParams[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

    std::array<D3D12_STATIC_SAMPLER_DESC, 2> staticSamplers{};
    staticSamplers[0].Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    staticSamplers[0].AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    staticSamplers[0].AddressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    staticSamplers[0].AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    staticSamplers[0].ShaderRegister = 0;
    staticSamplers[0].RegisterSpace = 0;
    staticSamplers[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    staticSamplers[0].MaxLOD = D3D12_FLOAT32_MAX;

    staticSamplers[1].Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    staticSamplers[1].AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    staticSamplers[1].AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    staticSamplers[1].AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    staticSamplers[1].ShaderRegister = 1;
    staticSamplers[1].RegisterSpace = 0;
    staticSamplers[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    staticSamplers[1].MaxLOD = D3D12_FLOAT32_MAX;

    D3D12_ROOT_SIGNATURE_DESC graphicsRootDesc{};
    graphicsRootDesc.NumParameters = static_cast<UINT>(std::size(graphicsParams));
    graphicsRootDesc.pParameters = graphicsParams;
    graphicsRootDesc.NumStaticSamplers = static_cast<UINT>(staticSamplers.size());
    graphicsRootDesc.pStaticSamplers = staticSamplers.data();
    graphicsRootDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    ComPtr<ID3DBlob> sig;
    ComPtr<ID3DBlob> sigErrors;
    HRESULT hr = D3D12SerializeRootSignature(
        &graphicsRootDesc,
        D3D_ROOT_SIGNATURE_VERSION_1,
        sig.ReleaseAndGetAddressOf(),
        sigErrors.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        std::string detail;
        if (sigErrors) {
            detail.assign(
                static_cast<const char*>(sigErrors->GetBufferPointer()),
                sigErrors->GetBufferSize());
        }
        error = "D3D12SerializeRootSignature(graphics) failed: " + detail;
        return false;
    }

    hr = m_device->CreateRootSignature(
        0,
        sig->GetBufferPointer(),
        sig->GetBufferSize(),
        IID_PPV_ARGS(m_rootSignature.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateRootSignature(graphics) failed", hr);
        return false;
    }

    D3D12_DESCRIPTOR_RANGE computeSrvRanges[2]{};
    computeSrvRanges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    computeSrvRanges[0].NumDescriptors = 16; // t0..t15
    computeSrvRanges[0].BaseShaderRegister = 0;
    computeSrvRanges[0].RegisterSpace = 0;
    computeSrvRanges[0].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
    computeSrvRanges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    computeSrvRanges[1].NumDescriptors = 3; // t19..t21
    computeSrvRanges[1].BaseShaderRegister = 19;
    computeSrvRanges[1].RegisterSpace = 0;
    computeSrvRanges[1].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_DESCRIPTOR_RANGE computeUavRange{};
    computeUavRange.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    computeUavRange.NumDescriptors = 4; // u0..u3
    computeUavRange.BaseShaderRegister = 0;
    computeUavRange.RegisterSpace = 0;
    computeUavRange.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER computeParams[3]{};
    computeParams[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    computeParams[0].Descriptor.ShaderRegister = 0;
    computeParams[0].Descriptor.RegisterSpace = 0;
    computeParams[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    computeParams[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    computeParams[1].DescriptorTable.NumDescriptorRanges = static_cast<UINT>(std::size(computeSrvRanges));
    computeParams[1].DescriptorTable.pDescriptorRanges = computeSrvRanges;
    computeParams[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    computeParams[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    computeParams[2].DescriptorTable.NumDescriptorRanges = 1;
    computeParams[2].DescriptorTable.pDescriptorRanges = &computeUavRange;
    computeParams[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_ROOT_SIGNATURE_DESC computeRootDesc{};
    computeRootDesc.NumParameters = static_cast<UINT>(std::size(computeParams));
    computeRootDesc.pParameters = computeParams;
    computeRootDesc.NumStaticSamplers = static_cast<UINT>(staticSamplers.size());
    computeRootDesc.pStaticSamplers = staticSamplers.data();
    computeRootDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;

    sig.Reset();
    sigErrors.Reset();
    hr = D3D12SerializeRootSignature(
        &computeRootDesc,
        D3D_ROOT_SIGNATURE_VERSION_1,
        sig.ReleaseAndGetAddressOf(),
        sigErrors.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        std::string detail;
        if (sigErrors) {
            detail.assign(
                static_cast<const char*>(sigErrors->GetBufferPointer()),
                sigErrors->GetBufferSize());
        }
        error = "D3D12SerializeRootSignature(compute) failed: " + detail;
        return false;
    }

    hr = m_device->CreateRootSignature(
        0,
        sig->GetBufferPointer(),
        sig->GetBufferSize(),
        IID_PPV_ARGS(m_computeRootSignature.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateRootSignature(compute) failed", hr);
        return false;
    }

    ComPtr<ID3DBlob> basicVs;
    ComPtr<ID3DBlob> basicPs;
    ComPtr<ID3DBlob> earthVs;
    ComPtr<ID3DBlob> earthPs;
    ComPtr<ID3DBlob> skyVs;
    ComPtr<ID3DBlob> skyPs;
    ComPtr<ID3DBlob> terrainVs;
    ComPtr<ID3DBlob> terrainPs;
    ComPtr<ID3DBlob> transCs;
    ComPtr<ID3DBlob> skyViewCs;
    ComPtr<ID3DBlob> multiCs;
    ComPtr<ID3DBlob> aerialCs;

    if (!CompileShader(m_shaderDir / "basic.hlsl", "VSMain", "vs_5_0", basicVs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "basic.hlsl", "PSMain", "ps_5_0", basicPs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "earth.hlsl", "VSMain", "vs_5_0", earthVs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "earth.hlsl", "PSMain", "ps_5_0", earthPs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "skybox.hlsl", "VSMain", "vs_5_0", skyVs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "skybox.hlsl", "PSMain", "ps_5_0", skyPs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "terrain.hlsl", "VSMain", "vs_5_0", terrainVs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "terrain.hlsl", "PSMain", "ps_5_0", terrainPs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "atmosphere_lut.hlsl", "CSGenerateTransmittance", "cs_5_0", transCs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "atmosphere_lut.hlsl", "CSGenerateSkyView", "cs_5_0", skyViewCs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "atmosphere_lut.hlsl", "CSGenerateMultipleScattering", "cs_5_0", multiCs, error)) {
        return false;
    }
    if (!CompileShader(m_shaderDir / "atmosphere_lut.hlsl", "CSGenerateAerialPerspective", "cs_5_0", aerialCs, error)) {
        return false;
    }

    D3D12_INPUT_ELEMENT_DESC input[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
    };

    D3D12_RASTERIZER_DESC rasterizer{};
    rasterizer.FillMode = D3D12_FILL_MODE_SOLID;
    rasterizer.CullMode = D3D12_CULL_MODE_BACK;
    rasterizer.FrontCounterClockwise = FALSE;
    rasterizer.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
    rasterizer.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
    rasterizer.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
    rasterizer.DepthClipEnable = TRUE;
    rasterizer.MultisampleEnable = FALSE;
    rasterizer.AntialiasedLineEnable = FALSE;
    rasterizer.ForcedSampleCount = 0;
    rasterizer.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;

    D3D12_BLEND_DESC blend{};
    blend.AlphaToCoverageEnable = FALSE;
    blend.IndependentBlendEnable = FALSE;
    const D3D12_RENDER_TARGET_BLEND_DESC defaultRtBlend{
        FALSE,
        FALSE,
        D3D12_BLEND_ONE,
        D3D12_BLEND_ZERO,
        D3D12_BLEND_OP_ADD,
        D3D12_BLEND_ONE,
        D3D12_BLEND_ZERO,
        D3D12_BLEND_OP_ADD,
        D3D12_LOGIC_OP_NOOP,
        D3D12_COLOR_WRITE_ENABLE_ALL};
    for (auto& rt : blend.RenderTarget) {
        rt = defaultRtBlend;
    }

    D3D12_DEPTH_STENCIL_DESC depthStencil{};
    depthStencil.DepthEnable = TRUE;
    depthStencil.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    depthStencil.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
    depthStencil.StencilEnable = FALSE;
    depthStencil.StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK;
    depthStencil.StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC pso{};
    pso.pRootSignature = m_rootSignature.Get();
    pso.InputLayout = {input, static_cast<UINT>(std::size(input))};
    pso.RasterizerState = rasterizer;
    pso.BlendState = blend;
    pso.DepthStencilState = depthStencil;
    pso.SampleMask = UINT_MAX;
    pso.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    pso.NumRenderTargets = 1;
    pso.RTVFormats[0] = kBackBufferFormat;
    pso.DSVFormat = kDepthFormat;
    pso.SampleDesc.Count = 1;

    pso.VS = {basicVs->GetBufferPointer(), basicVs->GetBufferSize()};
    pso.PS = {basicPs->GetBufferPointer(), basicPs->GetBufferSize()};

    hr = m_device->CreateGraphicsPipelineState(&pso, IID_PPV_ARGS(m_planePso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateGraphicsPipelineState(plane) failed", hr);
        return false;
    }

    pso.VS = {earthVs->GetBufferPointer(), earthVs->GetBufferSize()};
    pso.PS = {earthPs->GetBufferPointer(), earthPs->GetBufferSize()};
    pso.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;

    hr = m_device->CreateGraphicsPipelineState(&pso, IID_PPV_ARGS(m_earthPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateGraphicsPipelineState(earth) failed", hr);
        return false;
    }

    D3D12_GRAPHICS_PIPELINE_STATE_DESC skyPso = pso;
    skyPso.VS = {skyVs->GetBufferPointer(), skyVs->GetBufferSize()};
    skyPso.PS = {skyPs->GetBufferPointer(), skyPs->GetBufferSize()};
    skyPso.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    skyPso.DepthStencilState.DepthEnable = FALSE;
    skyPso.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;

    hr = m_device->CreateGraphicsPipelineState(&skyPso, IID_PPV_ARGS(m_skyboxPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateGraphicsPipelineState(skybox) failed", hr);
        return false;
    }

    D3D12_GRAPHICS_PIPELINE_STATE_DESC terrainPso = pso;
    terrainPso.VS = {terrainVs->GetBufferPointer(), terrainVs->GetBufferSize()};
    terrainPso.PS = {terrainPs->GetBufferPointer(), terrainPs->GetBufferSize()};
    terrainPso.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    terrainPso.DepthStencilState.DepthEnable = TRUE;
    terrainPso.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;

    hr = m_device->CreateGraphicsPipelineState(&terrainPso, IID_PPV_ARGS(m_terrainPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateGraphicsPipelineState(terrain) failed", hr);
        return false;
    }

    D3D12_GRAPHICS_PIPELINE_STATE_DESC terrainBlendPso = terrainPso;
    terrainBlendPso.BlendState.RenderTarget[0].BlendEnable = TRUE;
    terrainBlendPso.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_SRC_ALPHA;
    terrainBlendPso.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
    terrainBlendPso.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    terrainBlendPso.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
    terrainBlendPso.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA;
    terrainBlendPso.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    terrainBlendPso.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
    terrainBlendPso.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    hr = m_device->CreateGraphicsPipelineState(&terrainBlendPso, IID_PPV_ARGS(m_terrainBlendPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateGraphicsPipelineState(terrain blend) failed", hr);
        return false;
    }

    D3D12_COMPUTE_PIPELINE_STATE_DESC csDesc{};
    csDesc.pRootSignature = m_computeRootSignature.Get();
    csDesc.CS = {transCs->GetBufferPointer(), transCs->GetBufferSize()};
    hr = m_device->CreateComputePipelineState(&csDesc, IID_PPV_ARGS(m_transmittanceLutPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateComputePipelineState(transmittance) failed", hr);
        return false;
    }

    csDesc.CS = {skyViewCs->GetBufferPointer(), skyViewCs->GetBufferSize()};
    hr = m_device->CreateComputePipelineState(&csDesc, IID_PPV_ARGS(m_skyViewLutPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateComputePipelineState(sky view) failed", hr);
        return false;
    }

    csDesc.CS = {multiCs->GetBufferPointer(), multiCs->GetBufferSize()};
    hr = m_device->CreateComputePipelineState(&csDesc, IID_PPV_ARGS(m_multiScatteringLutPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateComputePipelineState(multiple scattering) failed", hr);
        return false;
    }

    csDesc.CS = {aerialCs->GetBufferPointer(), aerialCs->GetBufferSize()};
    hr = m_device->CreateComputePipelineState(&csDesc, IID_PPV_ARGS(m_aerialPerspectiveLutPso.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateComputePipelineState(aerial perspective) failed", hr);
        return false;
    }

    return true;
}

bool D3D12Renderer::CreateConstantBuffers(std::string& error) {
    m_sceneCbSize = Align256(sizeof(SceneConstants));
    m_objectCbStride = Align256(sizeof(ObjectConstants));

    for (UINT i = 0; i < kFrameCount; ++i) {
        D3D12_HEAP_PROPERTIES heapProps{};
        heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;

        D3D12_RESOURCE_DESC sceneDesc{};
        sceneDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
        sceneDesc.Width = m_sceneCbSize;
        sceneDesc.Height = 1;
        sceneDesc.DepthOrArraySize = 1;
        sceneDesc.MipLevels = 1;
        sceneDesc.SampleDesc.Count = 1;
        sceneDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

        HRESULT hr = m_device->CreateCommittedResource(
            &heapProps,
            D3D12_HEAP_FLAG_NONE,
            &sceneDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(m_sceneCb[i].ReleaseAndGetAddressOf()));
        if (FAILED(hr)) {
            error = HrMessage("CreateCommittedResource(scene CB) failed", hr);
            return false;
        }

        D3D12_RANGE readRange{};
        hr = m_sceneCb[i]->Map(0, &readRange, reinterpret_cast<void**>(&m_sceneCbMapped[i]));
        if (FAILED(hr)) {
            error = HrMessage("Map(scene CB) failed", hr);
            return false;
        }

        D3D12_RESOURCE_DESC objectDesc = sceneDesc;
        objectDesc.Width = m_objectCbStride * 4;

        hr = m_device->CreateCommittedResource(
            &heapProps,
            D3D12_HEAP_FLAG_NONE,
            &objectDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(m_objectCb[i].ReleaseAndGetAddressOf()));
        if (FAILED(hr)) {
            error = HrMessage("CreateCommittedResource(object CB) failed", hr);
            return false;
        }

        hr = m_objectCb[i]->Map(0, &readRange, reinterpret_cast<void**>(&m_objectCbMapped[i]));
        if (FAILED(hr)) {
            error = HrMessage("Map(object CB) failed", hr);
            return false;
        }
    }

    return true;
}

bool D3D12Renderer::CreateAtmosphereResources(std::string& error) {
    constexpr DXGI_FORMAT kLutFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
    constexpr UINT kTransW = 256;
    constexpr UINT kTransH = 64;
    constexpr UINT kSkyW = 256;
    constexpr UINT kSkyH = 128;
    constexpr UINT kMsW = 32;
    constexpr UINT kMsH = 32;
    constexpr UINT kAerialW = 32;
    constexpr UINT kAerialH = 32;
    constexpr UINT kAerialD = 32;
    constexpr D3D12_RESOURCE_STATES kInitialLutState =
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;

    D3D12_HEAP_PROPERTIES defaultHeap{};
    defaultHeap.Type = D3D12_HEAP_TYPE_DEFAULT;

    auto createLut2d = [&](UINT width, UINT height, Microsoft::WRL::ComPtr<ID3D12Resource>& outResource, const char* label) -> bool {
        D3D12_RESOURCE_DESC desc{};
        desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        desc.Width = width;
        desc.Height = height;
        desc.DepthOrArraySize = 1;
        desc.MipLevels = 1;
        desc.Format = kLutFormat;
        desc.SampleDesc.Count = 1;
        desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

        const HRESULT hr = m_device->CreateCommittedResource(
            &defaultHeap,
            D3D12_HEAP_FLAG_NONE,
            &desc,
            kInitialLutState,
            nullptr,
            IID_PPV_ARGS(outResource.ReleaseAndGetAddressOf()));
        if (FAILED(hr)) {
            error = HrMessage(label, hr);
            return false;
        }
        return true;
    };

    if (!createLut2d(kTransW, kTransH, m_transmittanceLut, "CreateCommittedResource(transmittance LUT) failed")) {
        return false;
    }
    if (!createLut2d(kSkyW, kSkyH, m_skyViewLut, "CreateCommittedResource(sky-view LUT) failed")) {
        return false;
    }
    if (!createLut2d(kMsW, kMsH, m_multipleScatteringLut, "CreateCommittedResource(multi-scattering LUT) failed")) {
        return false;
    }

    {
        D3D12_RESOURCE_DESC desc{};
        desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
        desc.Width = kAerialW;
        desc.Height = kAerialH;
        desc.DepthOrArraySize = static_cast<UINT16>(kAerialD);
        desc.MipLevels = 1;
        desc.Format = kLutFormat;
        desc.SampleDesc.Count = 1;
        desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

        const HRESULT hr = m_device->CreateCommittedResource(
            &defaultHeap,
            D3D12_HEAP_FLAG_NONE,
            &desc,
            kInitialLutState,
            nullptr,
            IID_PPV_ARGS(m_aerialPerspectiveLut.ReleaseAndGetAddressOf()));
        if (FAILED(hr)) {
            error = HrMessage("CreateCommittedResource(aerial perspective LUT) failed", hr);
            return false;
        }
    }

    D3D12_SHADER_RESOURCE_VIEW_DESC srv2d{};
    srv2d.Format = kLutFormat;
    srv2d.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv2d.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv2d.Texture2D.MipLevels = 1;
    m_device->CreateShaderResourceView(m_transmittanceLut.Get(), &srv2d, CpuSrv(kTransmittanceSrvIndex));
    m_device->CreateShaderResourceView(m_skyViewLut.Get(), &srv2d, CpuSrv(kSkyViewSrvIndex));
    m_device->CreateShaderResourceView(m_multipleScatteringLut.Get(), &srv2d, CpuSrv(kMultipleScatteringSrvIndex));

    D3D12_SHADER_RESOURCE_VIEW_DESC srv3d{};
    srv3d.Format = kLutFormat;
    srv3d.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    srv3d.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv3d.Texture3D.MipLevels = 1;
    m_device->CreateShaderResourceView(m_aerialPerspectiveLut.Get(), &srv3d, CpuSrv(kAerialPerspectiveSrvIndex));

    D3D12_UNORDERED_ACCESS_VIEW_DESC uav2d{};
    uav2d.Format = kLutFormat;
    uav2d.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    m_device->CreateUnorderedAccessView(m_transmittanceLut.Get(), nullptr, &uav2d, CpuSrv(kTransmittanceUavIndex));
    m_device->CreateUnorderedAccessView(m_skyViewLut.Get(), nullptr, &uav2d, CpuSrv(kSkyViewUavIndex));
    m_device->CreateUnorderedAccessView(m_multipleScatteringLut.Get(), nullptr, &uav2d, CpuSrv(kMultipleScatteringUavIndex));

    D3D12_UNORDERED_ACCESS_VIEW_DESC uav3d{};
    uav3d.Format = kLutFormat;
    uav3d.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    uav3d.Texture3D.WSize = kAerialD;
    m_device->CreateUnorderedAccessView(m_aerialPerspectiveLut.Get(), nullptr, &uav3d, CpuSrv(kAerialPerspectiveUavIndex));

    return true;
}

void D3D12Renderer::DispatchAtmosphereLuts(ID3D12GraphicsCommandList* commandList, const SceneConstants& sceneCb) {
    (void)sceneCb;
    if (!m_atmosphereEnabled) {
        return;
    }
    if (!m_computeRootSignature || !m_transmittanceLutPso || !m_skyViewLutPso || !m_multiScatteringLutPso || !m_aerialPerspectiveLutPso) {
        return;
    }
    if (!m_transmittanceLut || !m_skyViewLut || !m_multipleScatteringLut || !m_aerialPerspectiveLut) {
        return;
    }

    auto transition = [&](ID3D12Resource* resource, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource = resource;
        barrier.Transition.StateBefore = before;
        barrier.Transition.StateAfter = after;
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        commandList->ResourceBarrier(1, &barrier);
    };
    auto uavBarrier = [&](ID3D12Resource* resource) {
        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
        barrier.UAV.pResource = resource;
        commandList->ResourceBarrier(1, &barrier);
    };

    constexpr D3D12_RESOURCE_STATES kSrvState = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;

    transition(m_transmittanceLut.Get(), kSrvState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    transition(m_skyViewLut.Get(), kSrvState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    transition(m_multipleScatteringLut.Get(), kSrvState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    transition(m_aerialPerspectiveLut.Get(), kSrvState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    commandList->SetComputeRootSignature(m_computeRootSignature.Get());
    commandList->SetComputeRootConstantBufferView(0, m_sceneCb[m_frameIndex]->GetGPUVirtualAddress());
    commandList->SetComputeRootDescriptorTable(1, GpuSrv(kSrvTableStartIndex));
    commandList->SetComputeRootDescriptorTable(2, GpuSrv(kUavTableStartIndex));

    commandList->SetPipelineState(m_transmittanceLutPso.Get());
    commandList->Dispatch((256 + 7) / 8, (64 + 7) / 8, 1);
    uavBarrier(m_transmittanceLut.Get());

    commandList->SetPipelineState(m_multiScatteringLutPso.Get());
    commandList->Dispatch((32 + 7) / 8, (32 + 7) / 8, 1);
    uavBarrier(m_multipleScatteringLut.Get());

    commandList->SetPipelineState(m_skyViewLutPso.Get());
    commandList->Dispatch((256 + 7) / 8, (128 + 7) / 8, 1);
    uavBarrier(m_skyViewLut.Get());

    commandList->SetPipelineState(m_aerialPerspectiveLutPso.Get());
    commandList->Dispatch((32 + 3) / 4, (32 + 3) / 4, (32 + 3) / 4);
    uavBarrier(m_aerialPerspectiveLut.Get());

    transition(m_transmittanceLut.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, kSrvState);
    transition(m_skyViewLut.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, kSrvState);
    transition(m_multipleScatteringLut.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, kSrvState);
    transition(m_aerialPerspectiveLut.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, kSrvState);
}

bool D3D12Renderer::LoadLandmaskPixels(
    const std::filesystem::path& imagePath,
    std::vector<uint8_t>& pixels,
    uint32_t& outWidth,
    uint32_t& outHeight,
    std::string& error) {
    if (!m_wicFactory) {
        error = "WIC factory is not initialized";
        return false;
    }

    ComPtr<IWICBitmapDecoder> decoder;
    HRESULT hr = m_wicFactory->CreateDecoderFromFilename(
        imagePath.wstring().c_str(),
        nullptr,
        GENERIC_READ,
        WICDecodeMetadataCacheOnDemand,
        decoder.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        error = HrMessage("CreateDecoderFromFilename failed", hr);
        return false;
    }

    ComPtr<IWICBitmapFrameDecode> frame;
    hr = decoder->GetFrame(0, frame.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        error = HrMessage("WIC GetFrame failed", hr);
        return false;
    }

    UINT srcW = 0;
    UINT srcH = 0;
    frame->GetSize(&srcW, &srcH);

    // Downsample very large masks for quicker startup/memory use.
    const UINT maxDim = 4096;
    UINT dstW = srcW;
    UINT dstH = srcH;

    ComPtr<IWICBitmapSource> source;
    if (std::max(srcW, srcH) > maxDim) {
        const double scale = static_cast<double>(maxDim) / static_cast<double>(std::max(srcW, srcH));
        dstW = std::max(1u, static_cast<UINT>(std::floor(srcW * scale)));
        dstH = std::max(1u, static_cast<UINT>(std::floor(srcH * scale)));

        ComPtr<IWICBitmapScaler> scaler;
        hr = m_wicFactory->CreateBitmapScaler(scaler.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            error = HrMessage("CreateBitmapScaler failed", hr);
            return false;
        }
        hr = scaler->Initialize(frame.Get(), dstW, dstH, WICBitmapInterpolationModeLinear);
        if (FAILED(hr)) {
            error = HrMessage("BitmapScaler Initialize failed", hr);
            return false;
        }
        source = scaler;
    } else {
        source = frame;
    }

    ComPtr<IWICFormatConverter> converter;
    hr = m_wicFactory->CreateFormatConverter(converter.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        error = HrMessage("CreateFormatConverter failed", hr);
        return false;
    }

    hr = converter->Initialize(
        source.Get(),
        GUID_WICPixelFormat8bppGray,
        WICBitmapDitherTypeNone,
        nullptr,
        0.0,
        WICBitmapPaletteTypeCustom);
    if (FAILED(hr)) {
        error = HrMessage("FormatConverter Initialize failed", hr);
        return false;
    }

    pixels.resize(static_cast<size_t>(dstW) * static_cast<size_t>(dstH));
    hr = converter->CopyPixels(nullptr, dstW, static_cast<UINT>(pixels.size()), pixels.data());
    if (FAILED(hr)) {
        error = HrMessage("CopyPixels failed", hr);
        return false;
    }

    outWidth = dstW;
    outHeight = dstH;
    return true;
}

bool D3D12Renderer::LoadColorPixels(
    const std::filesystem::path& imagePath,
    std::vector<uint8_t>& pixelsRgba,
    uint32_t& outWidth,
    uint32_t& outHeight,
    uint32_t maxDim,
    std::string& error) {
    if (!m_wicFactory) {
        error = "WIC factory is not initialized";
        return false;
    }

    ComPtr<IWICBitmapDecoder> decoder;
    HRESULT hr = m_wicFactory->CreateDecoderFromFilename(
        imagePath.wstring().c_str(),
        nullptr,
        GENERIC_READ,
        WICDecodeMetadataCacheOnDemand,
        decoder.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        error = HrMessage("CreateDecoderFromFilename(color) failed", hr);
        return false;
    }

    ComPtr<IWICBitmapFrameDecode> frame;
    hr = decoder->GetFrame(0, frame.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        error = HrMessage("WIC GetFrame(color) failed", hr);
        return false;
    }

    UINT srcW = 0;
    UINT srcH = 0;
    frame->GetSize(&srcW, &srcH);

    ComPtr<IWICBitmapSource> source = frame;
    UINT dstW = srcW;
    UINT dstH = srcH;
    if (std::max(srcW, srcH) > maxDim) {
        const double scale = static_cast<double>(maxDim) / static_cast<double>(std::max(srcW, srcH));
        dstW = std::max(1u, static_cast<UINT>(std::floor(srcW * scale)));
        dstH = std::max(1u, static_cast<UINT>(std::floor(srcH * scale)));

        ComPtr<IWICBitmapScaler> scaler;
        hr = m_wicFactory->CreateBitmapScaler(scaler.ReleaseAndGetAddressOf());
        if (FAILED(hr)) {
            error = HrMessage("CreateBitmapScaler(color) failed", hr);
            return false;
        }
        hr = scaler->Initialize(frame.Get(), dstW, dstH, WICBitmapInterpolationModeLinear);
        if (FAILED(hr)) {
            error = HrMessage("BitmapScaler Initialize(color) failed", hr);
            return false;
        }
        source = scaler;
    }

    ComPtr<IWICFormatConverter> converter;
    hr = m_wicFactory->CreateFormatConverter(converter.ReleaseAndGetAddressOf());
    if (FAILED(hr)) {
        error = HrMessage("CreateFormatConverter(color) failed", hr);
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
        error = HrMessage("FormatConverter Initialize(color) failed", hr);
        return false;
    }

    const UINT stride = dstW * 4;
    pixelsRgba.resize(static_cast<size_t>(stride) * static_cast<size_t>(dstH));
    hr = converter->CopyPixels(nullptr, stride, static_cast<UINT>(pixelsRgba.size()), pixelsRgba.data());
    if (FAILED(hr)) {
        error = HrMessage("CopyPixels(color) failed", hr);
        return false;
    }

    outWidth = dstW;
    outHeight = dstH;
    return true;
}

bool D3D12Renderer::CreateSatelliteTextureFromPixels(
    ID3D12GraphicsCommandList* commandList,
    UINT srvIndex,
    ComPtr<ID3D12Resource>& outTexture,
    ComPtr<ID3D12Resource>& outUpload,
    const uint8_t* rgbaPixels,
    uint32_t width,
    uint32_t height,
    std::string& error) {
    if (rgbaPixels == nullptr || width == 0 || height == 0) {
        error = "CreateSatelliteTextureFromPixels received invalid pixel data";
        return false;
    }

    ComPtr<ID3D12Resource> texture;
    ComPtr<ID3D12Resource> upload;

    D3D12_HEAP_PROPERTIES defaultHeap{};
    defaultHeap.Type = D3D12_HEAP_TYPE_DEFAULT;

    D3D12_RESOURCE_DESC texDesc{};
    texDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    texDesc.Width = width;
    texDesc.Height = height;
    texDesc.DepthOrArraySize = 1;
    texDesc.MipLevels = 1;
    texDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    texDesc.SampleDesc.Count = 1;
    texDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;

    HRESULT hr = m_device->CreateCommittedResource(
        &defaultHeap,
        D3D12_HEAP_FLAG_NONE,
        &texDesc,
        D3D12_RESOURCE_STATE_COPY_DEST,
        nullptr,
        IID_PPV_ARGS(texture.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(model albedo texture) failed", hr);
        return false;
    }

    D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
    UINT numRows = 0;
    UINT64 rowBytes = 0;
    UINT64 uploadBytes = 0;
    m_device->GetCopyableFootprints(&texDesc, 0, 1, 0, &footprint, &numRows, &rowBytes, &uploadBytes);

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC uploadDesc{};
    uploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    uploadDesc.Width = uploadBytes;
    uploadDesc.Height = 1;
    uploadDesc.DepthOrArraySize = 1;
    uploadDesc.MipLevels = 1;
    uploadDesc.SampleDesc.Count = 1;
    uploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    hr = m_device->CreateCommittedResource(
        &uploadHeap,
        D3D12_HEAP_FLAG_NONE,
        &uploadDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(upload.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(model albedo upload) failed", hr);
        return false;
    }

    uint8_t* mapped = nullptr;
    D3D12_RANGE readRange{};
    hr = upload->Map(0, &readRange, reinterpret_cast<void**>(&mapped));
    if (FAILED(hr) || mapped == nullptr) {
        error = HrMessage("Map(model albedo upload) failed", hr);
        return false;
    }

    const uint8_t* src = rgbaPixels;
    uint8_t* dst = mapped + footprint.Offset;
    const UINT srcStride = width * 4;
    for (UINT row = 0; row < numRows; ++row) {
        std::memcpy(dst + row * footprint.Footprint.RowPitch, src + static_cast<size_t>(row) * srcStride, srcStride);
    }
    upload->Unmap(0, nullptr);

    D3D12_TEXTURE_COPY_LOCATION dstLoc{};
    dstLoc.pResource = texture.Get();
    dstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    dstLoc.SubresourceIndex = 0;

    D3D12_TEXTURE_COPY_LOCATION srcLoc{};
    srcLoc.pResource = upload.Get();
    srcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    srcLoc.PlacedFootprint = footprint;

    commandList->CopyTextureRegion(&dstLoc, 0, 0, 0, &srcLoc, nullptr);

    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition.pResource = texture.Get();
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    commandList->ResourceBarrier(1, &barrier);

    D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
    srv.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv.Texture2D.MipLevels = 1;
    m_device->CreateShaderResourceView(texture.Get(), &srv, CpuSrv(srvIndex));

    outTexture = std::move(texture);
    outUpload = std::move(upload);
    return true;
}

void D3D12Renderer::CreateSatelliteSrvForResource(ID3D12Resource* texture, UINT srvIndex) {
    D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
    srv.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv.Texture2D.MipLevels = 1;
    m_device->CreateShaderResourceView(texture, &srv, CpuSrv(srvIndex));
}

bool D3D12Renderer::CreatePlaneTextureFromPixels(
    ID3D12GraphicsCommandList* commandList,
    const uint8_t* rgbaPixels,
    uint32_t width,
    uint32_t height,
    std::string& error) {
    if (!CreateSatelliteTextureFromPixels(
            commandList,
            kModelAlbedoSrvIndex,
            m_modelAlbedoTexture,
            m_modelAlbedoUpload,
            rgbaPixels,
            width,
            height,
            error)) {
        return false;
    }
    m_hasModelAlbedoTexture = true;
    return true;
}

bool D3D12Renderer::CreateLandmaskTextureFromPixels(
    ID3D12GraphicsCommandList* commandList,
    const uint8_t* pixels,
    uint32_t width,
    uint32_t height,
    bool markLandmaskPresent,
    std::string& error) {
    D3D12_HEAP_PROPERTIES defaultHeap{};
    defaultHeap.Type = D3D12_HEAP_TYPE_DEFAULT;

    D3D12_RESOURCE_DESC texDesc{};
    texDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    texDesc.Width = width;
    texDesc.Height = height;
    texDesc.DepthOrArraySize = 1;
    texDesc.MipLevels = 1;
    texDesc.Format = DXGI_FORMAT_R8_UNORM;
    texDesc.SampleDesc.Count = 1;
    texDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;

    HRESULT hr = m_device->CreateCommittedResource(
        &defaultHeap,
        D3D12_HEAP_FLAG_NONE,
        &texDesc,
        D3D12_RESOURCE_STATE_COPY_DEST,
        nullptr,
        IID_PPV_ARGS(m_landmaskTexture.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(landmask texture) failed", hr);
        return false;
    }

    D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
    UINT numRows = 0;
    UINT64 rowBytes = 0;
    UINT64 uploadBytes = 0;
    m_device->GetCopyableFootprints(&texDesc, 0, 1, 0, &footprint, &numRows, &rowBytes, &uploadBytes);

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC uploadDesc{};
    uploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    uploadDesc.Width = uploadBytes;
    uploadDesc.Height = 1;
    uploadDesc.DepthOrArraySize = 1;
    uploadDesc.MipLevels = 1;
    uploadDesc.SampleDesc.Count = 1;
    uploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    hr = m_device->CreateCommittedResource(
        &uploadHeap,
        D3D12_HEAP_FLAG_NONE,
        &uploadDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(m_landmaskUpload.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(landmask upload) failed", hr);
        return false;
    }

    uint8_t* mapped = nullptr;
    D3D12_RANGE readRange{};
    hr = m_landmaskUpload->Map(0, &readRange, reinterpret_cast<void**>(&mapped));
    if (FAILED(hr)) {
        error = HrMessage("Map(landmask upload) failed", hr);
        return false;
    }

    const uint8_t* src = pixels;
    uint8_t* dst = mapped + footprint.Offset;
    for (UINT row = 0; row < numRows; ++row) {
        std::memcpy(dst + row * footprint.Footprint.RowPitch, src + row * width, width);
    }
    m_landmaskUpload->Unmap(0, nullptr);

    D3D12_TEXTURE_COPY_LOCATION dstLoc{};
    dstLoc.pResource = m_landmaskTexture.Get();
    dstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    dstLoc.SubresourceIndex = 0;

    D3D12_TEXTURE_COPY_LOCATION srcLoc{};
    srcLoc.pResource = m_landmaskUpload.Get();
    srcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    srcLoc.PlacedFootprint = footprint;

    commandList->CopyTextureRegion(&dstLoc, 0, 0, 0, &srcLoc, nullptr);

    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition.pResource = m_landmaskTexture.Get();
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    commandList->ResourceBarrier(1, &barrier);

    D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
    srv.Format = DXGI_FORMAT_R8_UNORM;
    srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv.Texture2D.MipLevels = 1;
    m_device->CreateShaderResourceView(m_landmaskTexture.Get(), &srv, CpuSrv(kLandmaskSrvIndex));

    m_hasLandmaskTexture = markLandmaskPresent;
    return true;
}

bool D3D12Renderer::CreateSkyboxTextureFromFaces(
    ID3D12GraphicsCommandList* commandList,
    const std::array<std::vector<uint8_t>, 6>& facePixels,
    uint32_t width,
    uint32_t height,
    std::string& error) {
    D3D12_HEAP_PROPERTIES defaultHeap{};
    defaultHeap.Type = D3D12_HEAP_TYPE_DEFAULT;

    D3D12_RESOURCE_DESC texDesc{};
    texDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    texDesc.Width = width;
    texDesc.Height = height;
    texDesc.DepthOrArraySize = 6;
    texDesc.MipLevels = 1;
    texDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    texDesc.SampleDesc.Count = 1;
    texDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    texDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

    HRESULT hr = m_device->CreateCommittedResource(
        &defaultHeap,
        D3D12_HEAP_FLAG_NONE,
        &texDesc,
        D3D12_RESOURCE_STATE_COPY_DEST,
        nullptr,
        IID_PPV_ARGS(m_skyboxTexture.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(skybox texture) failed", hr);
        return false;
    }

    std::array<D3D12_PLACED_SUBRESOURCE_FOOTPRINT, 6> footprints{};
    std::array<UINT, 6> numRows{};
    std::array<UINT64, 6> rowSizes{};
    UINT64 uploadBytes = 0;
    m_device->GetCopyableFootprints(
        &texDesc,
        0,
        6,
        0,
        footprints.data(),
        numRows.data(),
        rowSizes.data(),
        &uploadBytes);

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC uploadDesc{};
    uploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    uploadDesc.Width = uploadBytes;
    uploadDesc.Height = 1;
    uploadDesc.DepthOrArraySize = 1;
    uploadDesc.MipLevels = 1;
    uploadDesc.SampleDesc.Count = 1;
    uploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    hr = m_device->CreateCommittedResource(
        &uploadHeap,
        D3D12_HEAP_FLAG_NONE,
        &uploadDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(m_skyboxUpload.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(skybox upload) failed", hr);
        return false;
    }

    uint8_t* mapped = nullptr;
    D3D12_RANGE readRange{};
    hr = m_skyboxUpload->Map(0, &readRange, reinterpret_cast<void**>(&mapped));
    if (FAILED(hr)) {
        error = HrMessage("Map(skybox upload) failed", hr);
        return false;
    }

    for (UINT face = 0; face < 6; ++face) {
        const uint8_t* src = facePixels[face].data();
        uint8_t* dst = mapped + footprints[face].Offset;
        for (UINT row = 0; row < numRows[face]; ++row) {
            std::memcpy(dst + row * footprints[face].Footprint.RowPitch, src + row * width * 4, width * 4);
        }
    }
    m_skyboxUpload->Unmap(0, nullptr);

    for (UINT face = 0; face < 6; ++face) {
        D3D12_TEXTURE_COPY_LOCATION dstLoc{};
        dstLoc.pResource = m_skyboxTexture.Get();
        dstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dstLoc.SubresourceIndex = face;

        D3D12_TEXTURE_COPY_LOCATION srcLoc{};
        srcLoc.pResource = m_skyboxUpload.Get();
        srcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        srcLoc.PlacedFootprint = footprints[face];

        commandList->CopyTextureRegion(&dstLoc, 0, 0, 0, &srcLoc, nullptr);
    }

    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition.pResource = m_skyboxTexture.Get();
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    commandList->ResourceBarrier(1, &barrier);

    D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
    srv.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv.TextureCube.MipLevels = 1;
    m_device->CreateShaderResourceView(m_skyboxTexture.Get(), &srv, CpuSrv(kSkyboxSrvIndex));
    return true;
}

bool D3D12Renderer::CreateLandmaskTexture(ID3D12GraphicsCommandList* commandList, std::string& error) {
    std::vector<std::filesystem::path> candidates = {
        m_assetDir / "textures" / "landmask_16k.png",
        m_assetDir / "textures" / "landmask.png",
        m_assetDir / "landmask_16k.png",
        m_assetDir / "landmask.png",
    };

    for (const auto& p : candidates) {
        if (!std::filesystem::exists(p)) {
            continue;
        }

        std::vector<uint8_t> pixels;
        uint32_t width = 0;
        uint32_t height = 0;
        std::string loadError;
        if (LoadLandmaskPixels(p, pixels, width, height, loadError)) {
            return CreateLandmaskTextureFromPixels(commandList, pixels.data(), width, height, true, error);
        }
    }

    // Fallback: 1x1 ocean pixel.
    const uint8_t ocean = 0;
    return CreateLandmaskTextureFromPixels(commandList, &ocean, 1, 1, false, error);
}

bool D3D12Renderer::CreateSkyboxTexture(ID3D12GraphicsCommandList* commandList, std::string& error) {
    const std::array<std::filesystem::path, 6> facePaths = {
        m_assetDir / "textures" / "skybox" / "daylight_posx.png", // +X right
        m_assetDir / "textures" / "skybox" / "daylight_negx.png", // -X left
        m_assetDir / "textures" / "skybox" / "daylight_posy.png", // +Y top
        m_assetDir / "textures" / "skybox" / "daylight_negy.png", // -Y bottom
        m_assetDir / "textures" / "skybox" / "daylight_posz.png", // +Z front
        m_assetDir / "textures" / "skybox" / "daylight_negz.png", // -Z back
    };

    bool allFacesPresent = true;
    for (const auto& path : facePaths) {
        if (!std::filesystem::exists(path)) {
            allFacesPresent = false;
            break;
        }
    }

    if (allFacesPresent) {
        std::array<std::vector<uint8_t>, 6> faces{};
        uint32_t width = 0;
        uint32_t height = 0;
        for (size_t i = 0; i < facePaths.size(); ++i) {
            uint32_t w = 0;
            uint32_t h = 0;
            std::string loadError;
            if (!LoadColorPixels(facePaths[i], faces[i], w, h, 4096, loadError)) {
                allFacesPresent = false;
                break;
            }
            if (i == 0) {
                width = w;
                height = h;
            } else if (w != width || h != height) {
                allFacesPresent = false;
                break;
            }
        }

        if (allFacesPresent && width > 0 && height > 0) {
            return CreateSkyboxTextureFromFaces(commandList, faces, width, height, error);
        }
    }

    // Fallback day-like cubemap if files are missing.
    std::array<std::vector<uint8_t>, 6> fallback{};
    fallback[0] = {146, 187, 244, 255};
    fallback[1] = {146, 187, 244, 255};
    fallback[2] = {120, 170, 242, 255};
    fallback[3] = {100, 145, 220, 255};
    fallback[4] = {164, 200, 250, 255};
    fallback[5] = {164, 200, 250, 255};

    return CreateSkyboxTextureFromFaces(commandList, fallback, 1, 1, error);
}

bool D3D12Renderer::CreateEarthAlbedoTexture(ID3D12GraphicsCommandList* commandList, std::string& error) {
    // Optional global equirectangular albedo used for terrain coloring (Blue Marble world.*_geo.tif).
    // Missing/failed loads fall back to a 1x1 neutral pixel so the game keeps running.
    error.clear();

    const std::filesystem::path blueMarbleRel = std::filesystem::path("textures") / "satellite" / "world.200406.3x21600x21600.A1_geo.tif";
    std::filesystem::path found;
    auto toLowerAscii = [](std::string s) {
        for (char& c : s) {
            if (c >= 'A' && c <= 'Z') {
                c = static_cast<char>(c - 'A' + 'a');
            }
        }
        return s;
    };

    auto consider = [&](const std::filesystem::path& p) {
        if (!found.empty()) {
            return;
        }
        std::error_code ec;
        if (std::filesystem::exists(p, ec) && !ec) {
            found = p;
        }
    };

    auto considerWorldGeoInDir = [&](const std::filesystem::path& dir) {
        if (!found.empty()) {
            return;
        }
        std::error_code ec;
        if (!std::filesystem::exists(dir, ec) || ec) {
            return;
        }
        for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
            if (ec) {
                break;
            }
            if (!entry.is_regular_file(ec) || ec) {
                continue;
            }
            const std::string nameLower = toLowerAscii(entry.path().filename().string());
            const bool isWorldPrefix = nameLower.rfind("world.", 0) == 0;
            const bool hasGeoSuffix =
                (nameLower.size() >= 8 && nameLower.compare(nameLower.size() - 8, 8, "_geo.tif") == 0) ||
                (nameLower.size() >= 9 && nameLower.compare(nameLower.size() - 9, 9, "_geo.tiff") == 0);
            if (isWorldPrefix && hasGeoSuffix) {
                found = entry.path();
                return;
            }
        }
    };

    // Prefer an assets-based canonical location.
    consider(m_assetDir / blueMarbleRel);
    considerWorldGeoInDir(m_assetDir / "textures" / "satellite");

    // Also search upwards from the working directory so running from build/Debug still finds repo-root world.*_geo.tif.
    std::filesystem::path dir = std::filesystem::current_path();
    for (int i = 0; i < 8 && !dir.empty(); ++i) {
        consider(dir / blueMarbleRel);
        considerWorldGeoInDir(dir);
        considerWorldGeoInDir(dir / "assets" / "textures" / "satellite");
        const std::filesystem::path parent = dir.parent_path();
        if (parent == dir) {
            break;
        }
        dir = parent;
    }

    std::vector<uint8_t> basePixels;
    uint32_t width = 1;
    uint32_t height = 1;
    bool loaded = false;
    std::array<double, 6> geoTransform{};
    bool hasGeoTransform = false;
    m_earthAlbedoSourcePath.clear();
    m_hasEarthAlbedoSourceFile = false;
    m_earthAlbedoBoundsLonLat = {-180.0f, 180.0f, -90.0f, 90.0f};
    m_earthAlbedoWrapLon = 1.0f;
    if (!found.empty()) {
        std::string loadError;
        // Downscale to keep memory reasonable. This is a single global/large texture.
        const uint32_t maxDim = 6144;

        const std::wstring ext = found.extension().wstring();
        const bool isTiff = (ext == L".tif") || (ext == L".tiff") || (ext == L".TIF") || (ext == L".TIFF");
        if (isTiff) {
            if (LoadGeoTiffRgbWithGeoTransform(found, maxDim, basePixels, width, height, geoTransform, hasGeoTransform, loadError)) {
                loaded = true;
            }
        } else {
            if (LoadColorPixels(found, basePixels, width, height, maxDim, loadError)) {
                loaded = true;
            }
        }

        if (loaded && hasGeoTransform) {
            // Compute bounds from geotransform (lon/lat degrees).
            const double gt0 = geoTransform[0];
            const double gt1 = geoTransform[1];
            const double gt2 = geoTransform[2];
            const double gt3 = geoTransform[3];
            const double gt4 = geoTransform[4];
            const double gt5 = geoTransform[5];

            const double w = static_cast<double>(width);
            const double h = static_cast<double>(height);
            const double lonA = gt0;
            const double latA = gt3;
            const double lonB = gt0 + gt1 * w + gt2 * h;
            const double latB = gt3 + gt4 * w + gt5 * h;

            const double lonW = std::min(lonA, lonB);
            const double lonE = std::max(lonA, lonB);
            const double latS = std::min(latA, latB);
            const double latN = std::max(latA, latB);

            m_earthAlbedoBoundsLonLat = {
                static_cast<float>(lonW),
                static_cast<float>(lonE),
                static_cast<float>(latS),
                static_cast<float>(latN),
            };

            const double lonSpan = lonE - lonW;
            m_earthAlbedoWrapLon = (std::abs(lonSpan - 360.0) < 0.1) ? 1.0f : 0.0f;
        }
    }

    if (!loaded) {
        // Neutral mid-gray; used when imagery is missing.
        basePixels = {128, 128, 128, 255};
        width = 1;
        height = 1;
    } else {
        m_hasEarthAlbedoSourceFile = true;
        m_earthAlbedoSourcePath = found;
    }

    // Build a CPU mip chain to reduce shimmer (hardware will pick proper mips at distance).
    std::vector<std::vector<uint8_t>> mipPixels;
    std::vector<uint32_t> mipW;
    std::vector<uint32_t> mipH;
    mipPixels.emplace_back(std::move(basePixels));
    mipW.push_back(width);
    mipH.push_back(height);

    uint32_t curW = width;
    uint32_t curH = height;
    while (curW > 1 || curH > 1) {
        const uint32_t nextW = std::max(1u, curW / 2);
        const uint32_t nextH = std::max(1u, curH / 2);
        std::vector<uint8_t> next(static_cast<size_t>(nextW) * static_cast<size_t>(nextH) * 4u);

        const std::vector<uint8_t>& src = mipPixels.back();
        for (uint32_t y = 0; y < nextH; ++y) {
            const uint32_t sy0 = std::min(curH - 1, y * 2);
            const uint32_t sy1 = std::min(curH - 1, y * 2 + 1);
            for (uint32_t x = 0; x < nextW; ++x) {
                const uint32_t sx0 = std::min(curW - 1, x * 2);
                const uint32_t sx1 = std::min(curW - 1, x * 2 + 1);

                const uint32_t idx00 = (sy0 * curW + sx0) * 4;
                const uint32_t idx10 = (sy0 * curW + sx1) * 4;
                const uint32_t idx01 = (sy1 * curW + sx0) * 4;
                const uint32_t idx11 = (sy1 * curW + sx1) * 4;

                const uint32_t dst = (y * nextW + x) * 4;
                for (uint32_t c = 0; c < 4; ++c) {
                    const uint32_t sum =
                        static_cast<uint32_t>(src[idx00 + c]) +
                        static_cast<uint32_t>(src[idx10 + c]) +
                        static_cast<uint32_t>(src[idx01 + c]) +
                        static_cast<uint32_t>(src[idx11 + c]);
                    next[dst + c] = static_cast<uint8_t>(sum / 4u);
                }
            }
        }

        mipPixels.emplace_back(std::move(next));
        mipW.push_back(nextW);
        mipH.push_back(nextH);
        curW = nextW;
        curH = nextH;
    }

    const UINT mipLevels = static_cast<UINT>(mipPixels.size());

    D3D12_HEAP_PROPERTIES defaultHeap{};
    defaultHeap.Type = D3D12_HEAP_TYPE_DEFAULT;

    D3D12_RESOURCE_DESC texDesc{};
    texDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    texDesc.Width = mipW[0];
    texDesc.Height = mipH[0];
    texDesc.DepthOrArraySize = 1;
    texDesc.MipLevels = static_cast<UINT16>(mipLevels);
    texDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    texDesc.SampleDesc.Count = 1;
    texDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;

    HRESULT hr = m_device->CreateCommittedResource(
        &defaultHeap,
        D3D12_HEAP_FLAG_NONE,
        &texDesc,
        D3D12_RESOURCE_STATE_COPY_DEST,
        nullptr,
        IID_PPV_ARGS(m_earthAlbedoTexture.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(earth albedo) failed", hr);
        return false;
    }

    std::vector<D3D12_PLACED_SUBRESOURCE_FOOTPRINT> footprints(mipLevels);
    std::vector<UINT> numRows(mipLevels);
    std::vector<UINT64> rowBytes(mipLevels);
    UINT64 totalBytes = 0;
    m_device->GetCopyableFootprints(&texDesc, 0, mipLevels, 0, footprints.data(), numRows.data(), rowBytes.data(), &totalBytes);

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC uploadDesc{};
    uploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    uploadDesc.Width = totalBytes;
    uploadDesc.Height = 1;
    uploadDesc.DepthOrArraySize = 1;
    uploadDesc.MipLevels = 1;
    uploadDesc.Format = DXGI_FORMAT_UNKNOWN;
    uploadDesc.SampleDesc.Count = 1;
    uploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    hr = m_device->CreateCommittedResource(
        &uploadHeap,
        D3D12_HEAP_FLAG_NONE,
        &uploadDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(m_earthAlbedoUpload.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(earth albedo upload) failed", hr);
        return false;
    }

    uint8_t* mapped = nullptr;
    hr = m_earthAlbedoUpload->Map(0, nullptr, reinterpret_cast<void**>(&mapped));
    if (FAILED(hr) || mapped == nullptr) {
        error = HrMessage("Map(earth albedo upload) failed", hr);
        return false;
    }

    for (UINT mip = 0; mip < mipLevels; ++mip) {
        const uint32_t w = mipW[mip];
        const uint32_t h = mipH[mip];
        const uint8_t* src = mipPixels[mip].data();
        const UINT srcStride = w * 4;

        const auto& fp = footprints[mip];
        uint8_t* dstBase = mapped + fp.Offset;
        const UINT dstStride = fp.Footprint.RowPitch;
        for (uint32_t y = 0; y < h; ++y) {
            std::memcpy(dstBase + static_cast<size_t>(y) * dstStride, src + static_cast<size_t>(y) * srcStride, srcStride);
        }
    }

    m_earthAlbedoUpload->Unmap(0, nullptr);

    for (UINT mip = 0; mip < mipLevels; ++mip) {
        D3D12_TEXTURE_COPY_LOCATION dstLoc{};
        dstLoc.pResource = m_earthAlbedoTexture.Get();
        dstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dstLoc.SubresourceIndex = mip;

        D3D12_TEXTURE_COPY_LOCATION srcLoc{};
        srcLoc.pResource = m_earthAlbedoUpload.Get();
        srcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        srcLoc.PlacedFootprint = footprints[mip];

        commandList->CopyTextureRegion(&dstLoc, 0, 0, 0, &srcLoc, nullptr);
    }

    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition.pResource = m_earthAlbedoTexture.Get();
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    commandList->ResourceBarrier(1, &barrier);

    D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
    srv.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv.Texture2D.MipLevels = mipLevels;
    m_device->CreateShaderResourceView(m_earthAlbedoTexture.Get(), &srv, CpuSrv(kEarthAlbedoSrvIndex));

    return true;
}

bool D3D12Renderer::CreateWorldStreamingResources(ID3D12GraphicsCommandList* commandList, std::string& error) {
    if (!commandList) {
        error = "CreateWorldStreamingResources received null command list";
        return false;
    }

    m_worldSatellitePageTableCpu.assign(
        static_cast<size_t>(kWorldSatellitePageTableWidth) * static_cast<size_t>(kWorldSatellitePageTableHeight),
        0u);
    m_worldSatellitePageKeyCpu.assign(
        static_cast<size_t>(kWorldSatellitePageTableWidth) * static_cast<size_t>(kWorldSatellitePageTableHeight) * 2u,
        0u);

    D3D12_HEAP_PROPERTIES defaultHeap{};
    defaultHeap.Type = D3D12_HEAP_TYPE_DEFAULT;

    D3D12_RESOURCE_DESC atlasDesc{};
    atlasDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    atlasDesc.Width = static_cast<UINT64>(kWorldSatelliteTileSize) * static_cast<UINT64>(kWorldSatelliteAtlasPagesX);
    atlasDesc.Height = kWorldSatelliteTileSize * kWorldSatelliteAtlasPagesY;
    atlasDesc.DepthOrArraySize = 1;
    atlasDesc.MipLevels = 1;
    atlasDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    atlasDesc.SampleDesc.Count = 1;
    atlasDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    atlasDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

    HRESULT hr = m_device->CreateCommittedResource(
        &defaultHeap,
        D3D12_HEAP_FLAG_NONE,
        &atlasDesc,
        D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
        nullptr,
        IID_PPV_ARGS(m_worldSatelliteAtlasTexture.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(world satellite atlas) failed", hr);
        return false;
    }

    D3D12_RESOURCE_DESC pageTableDesc{};
    pageTableDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    pageTableDesc.Width = kWorldSatellitePageTableWidth;
    pageTableDesc.Height = kWorldSatellitePageTableHeight;
    pageTableDesc.DepthOrArraySize = 1;
    pageTableDesc.MipLevels = 1;
    pageTableDesc.Format = DXGI_FORMAT_R32_UINT;
    pageTableDesc.SampleDesc.Count = 1;
    pageTableDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    pageTableDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

    hr = m_device->CreateCommittedResource(
        &defaultHeap,
        D3D12_HEAP_FLAG_NONE,
        &pageTableDesc,
        D3D12_RESOURCE_STATE_COPY_DEST,
        nullptr,
        IID_PPV_ARGS(m_worldSatellitePageTableTexture.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(world satellite page table) failed", hr);
        return false;
    }

    D3D12_RESOURCE_DESC pageKeyDesc{};
    pageKeyDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    pageKeyDesc.Width = kWorldSatellitePageTableWidth;
    pageKeyDesc.Height = kWorldSatellitePageTableHeight;
    pageKeyDesc.DepthOrArraySize = 1;
    pageKeyDesc.MipLevels = 1;
    pageKeyDesc.Format = DXGI_FORMAT_R32G32_UINT;
    pageKeyDesc.SampleDesc.Count = 1;
    pageKeyDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    pageKeyDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

    hr = m_device->CreateCommittedResource(
        &defaultHeap,
        D3D12_HEAP_FLAG_NONE,
        &pageKeyDesc,
        D3D12_RESOURCE_STATE_COPY_DEST,
        nullptr,
        IID_PPV_ARGS(m_worldSatellitePageKeyTexture.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = HrMessage("CreateCommittedResource(world satellite page key texture) failed", hr);
        return false;
    }

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;
    auto uploadZeroTexture = [&](ID3D12Resource* dstTexture, const D3D12_RESOURCE_DESC& desc, ComPtr<ID3D12Resource>& outUpload,
                                 const char* createLabel, const char* mapLabel) -> bool {
        D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
        UINT numRows = 0;
        UINT64 rowBytes = 0;
        UINT64 uploadBytes = 0;
        m_device->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, &numRows, &rowBytes, &uploadBytes);

        D3D12_RESOURCE_DESC uploadDesc{};
        uploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
        uploadDesc.Width = uploadBytes;
        uploadDesc.Height = 1;
        uploadDesc.DepthOrArraySize = 1;
        uploadDesc.MipLevels = 1;
        uploadDesc.SampleDesc.Count = 1;
        uploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

        HRESULT localHr = m_device->CreateCommittedResource(
            &uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &uploadDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(outUpload.ReleaseAndGetAddressOf()));
        if (FAILED(localHr)) {
            error = HrMessage(createLabel, localHr);
            return false;
        }

        uint8_t* mapped = nullptr;
        D3D12_RANGE readRange{};
        localHr = outUpload->Map(0, &readRange, reinterpret_cast<void**>(&mapped));
        if (FAILED(localHr) || mapped == nullptr) {
            error = HrMessage(mapLabel, localHr);
            return false;
        }
        std::memset(mapped, 0, static_cast<size_t>(uploadBytes));
        outUpload->Unmap(0, nullptr);

        D3D12_TEXTURE_COPY_LOCATION dstLoc{};
        dstLoc.pResource = dstTexture;
        dstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dstLoc.SubresourceIndex = 0;

        D3D12_TEXTURE_COPY_LOCATION srcLoc{};
        srcLoc.pResource = outUpload.Get();
        srcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        srcLoc.PlacedFootprint = footprint;
        commandList->CopyTextureRegion(&dstLoc, 0, 0, 0, &srcLoc, nullptr);
        return true;
    };

    ComPtr<ID3D12Resource> pageTableUpload;
    if (!uploadZeroTexture(
            m_worldSatellitePageTableTexture.Get(),
            pageTableDesc,
            pageTableUpload,
            "CreateCommittedResource(world page table upload) failed",
            "Map(world page table upload) failed")) {
        return false;
    }

    ComPtr<ID3D12Resource> pageKeyUpload;
    if (!uploadZeroTexture(
            m_worldSatellitePageKeyTexture.Get(),
            pageKeyDesc,
            pageKeyUpload,
            "CreateCommittedResource(world page key upload) failed",
            "Map(world page key upload) failed")) {
        return false;
    }

    D3D12_RESOURCE_BARRIER barriers[2]{};
    barriers[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barriers[0].Transition.pResource = m_worldSatellitePageTableTexture.Get();
    barriers[0].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barriers[0].Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    barriers[0].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    barriers[1].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barriers[1].Transition.pResource = m_worldSatellitePageKeyTexture.Get();
    barriers[1].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barriers[1].Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    barriers[1].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    commandList->ResourceBarrier(2, barriers);

    D3D12_SHADER_RESOURCE_VIEW_DESC atlasSrv{};
    atlasSrv.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    atlasSrv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    atlasSrv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    atlasSrv.Texture2D.MipLevels = 1;
    m_device->CreateShaderResourceView(m_worldSatelliteAtlasTexture.Get(), &atlasSrv, CpuSrv(kWorldSatelliteAtlasSrvIndex));

    D3D12_SHADER_RESOURCE_VIEW_DESC pageSrv{};
    pageSrv.Format = DXGI_FORMAT_R32_UINT;
    pageSrv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    pageSrv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    pageSrv.Texture2D.MipLevels = 1;
    m_device->CreateShaderResourceView(m_worldSatellitePageTableTexture.Get(), &pageSrv, CpuSrv(kWorldSatellitePageTableSrvIndex));

    D3D12_SHADER_RESOURCE_VIEW_DESC pageKeySrv{};
    pageKeySrv.Format = DXGI_FORMAT_R32G32_UINT;
    pageKeySrv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    pageKeySrv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    pageKeySrv.Texture2D.MipLevels = 1;
    m_device->CreateShaderResourceView(m_worldSatellitePageKeyTexture.Get(), &pageKeySrv, CpuSrv(kWorldSatellitePageKeySrvIndex));

    m_worldStreamingResourcesReady = true;
    m_worldStreamingStats.resourcesReady = true;
    DeferredResource deferredPageTable{};
    deferredPageTable.resource = std::move(pageTableUpload);
    deferredPageTable.safeFenceValue = m_fenceValue;
    m_retiredResources.push_back(std::move(deferredPageTable));
    DeferredResource deferredPageKey{};
    deferredPageKey.resource = std::move(pageKeyUpload);
    deferredPageKey.safeFenceValue = m_fenceValue;
    m_retiredResources.push_back(std::move(deferredPageKey));
    return true;
}

bool D3D12Renderer::UploadWorldLockedSatelliteData(
    const std::vector<WorldAtlasPageUpload>& pageUploads,
    const std::vector<WorldPageTableUpdate>& pageTableUpdates,
    std::string& error) {
    error.clear();
    if (!m_worldStreamingResourcesReady || !m_worldSatelliteAtlasTexture || !m_worldSatellitePageTableTexture ||
        !m_worldSatellitePageKeyTexture) {
        error = "World satellite streaming resources are not initialized";
        m_worldStreamingStats.uploadFailures += 1;
        return false;
    }
    if (!m_worldUploadAllocator || !m_worldUploadCommandList || !m_fence) {
        error = "World upload command resources are not initialized";
        m_worldStreamingStats.uploadFailures += 1;
        return false;
    }
    if (pageUploads.empty() && pageTableUpdates.empty()) {
        return true;
    }

    CleanupDeferredTerrainResources();

    if (FAILED(m_worldUploadAllocator->Reset())) {
        error = "World upload allocator reset failed";
        m_worldStreamingStats.uploadFailures += 1;
        return false;
    }
    if (FAILED(m_worldUploadCommandList->Reset(m_worldUploadAllocator.Get(), nullptr))) {
        error = "World upload command list reset failed";
        m_worldStreamingStats.uploadFailures += 1;
        return false;
    }

    std::vector<ComPtr<ID3D12Resource>> transientUploads;
    transientUploads.reserve(pageUploads.size() + 4);
    uint64_t uploadedBytes = 0;

    if (!pageUploads.empty()) {
        D3D12_RESOURCE_BARRIER toCopy{};
        toCopy.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        toCopy.Transition.pResource = m_worldSatelliteAtlasTexture.Get();
        toCopy.Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        toCopy.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
        toCopy.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        m_worldUploadCommandList->ResourceBarrier(1, &toCopy);

        for (const WorldAtlasPageUpload& upload : pageUploads) {
            if (!upload.rgbaPixels) {
                continue;
            }
            if (upload.atlasPageX >= kWorldSatelliteAtlasPagesX || upload.atlasPageY >= kWorldSatelliteAtlasPagesY) {
                continue;
            }

            const UINT rowBytes = kWorldSatelliteTileSize * 4u;
            const UINT rowPitch =
                ((rowBytes + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1u) / D3D12_TEXTURE_DATA_PITCH_ALIGNMENT) * D3D12_TEXTURE_DATA_PITCH_ALIGNMENT;
            const UINT64 uploadBytes = static_cast<UINT64>(rowPitch) * static_cast<UINT64>(kWorldSatelliteTileSize);

            D3D12_HEAP_PROPERTIES uploadHeap{};
            uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;

            D3D12_RESOURCE_DESC uploadDesc{};
            uploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
            uploadDesc.Width = uploadBytes;
            uploadDesc.Height = 1;
            uploadDesc.DepthOrArraySize = 1;
            uploadDesc.MipLevels = 1;
            uploadDesc.SampleDesc.Count = 1;
            uploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

            ComPtr<ID3D12Resource> uploadBuffer;
            HRESULT hr = m_device->CreateCommittedResource(
                &uploadHeap,
                D3D12_HEAP_FLAG_NONE,
                &uploadDesc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(uploadBuffer.ReleaseAndGetAddressOf()));
            if (FAILED(hr)) {
                error = HrMessage("CreateCommittedResource(world atlas upload) failed", hr);
                m_worldStreamingStats.uploadFailures += 1;
                return false;
            }

            uint8_t* mapped = nullptr;
            D3D12_RANGE readRange{};
            hr = uploadBuffer->Map(0, &readRange, reinterpret_cast<void**>(&mapped));
            if (FAILED(hr) || mapped == nullptr) {
                error = HrMessage("Map(world atlas upload) failed", hr);
                m_worldStreamingStats.uploadFailures += 1;
                return false;
            }
            for (uint32_t row = 0; row < kWorldSatelliteTileSize; ++row) {
                const uint8_t* src = upload.rgbaPixels + static_cast<size_t>(row) * static_cast<size_t>(rowBytes);
                std::memcpy(mapped + static_cast<size_t>(row) * static_cast<size_t>(rowPitch), src, rowBytes);
            }
            uploadBuffer->Unmap(0, nullptr);

            D3D12_TEXTURE_COPY_LOCATION dstLoc{};
            dstLoc.pResource = m_worldSatelliteAtlasTexture.Get();
            dstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            dstLoc.SubresourceIndex = 0;

            D3D12_TEXTURE_COPY_LOCATION srcLoc{};
            srcLoc.pResource = uploadBuffer.Get();
            srcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            srcLoc.PlacedFootprint.Offset = 0;
            srcLoc.PlacedFootprint.Footprint.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
            srcLoc.PlacedFootprint.Footprint.Width = kWorldSatelliteTileSize;
            srcLoc.PlacedFootprint.Footprint.Height = kWorldSatelliteTileSize;
            srcLoc.PlacedFootprint.Footprint.Depth = 1;
            srcLoc.PlacedFootprint.Footprint.RowPitch = rowPitch;

            const UINT dstX = upload.atlasPageX * kWorldSatelliteTileSize;
            const UINT dstY = upload.atlasPageY * kWorldSatelliteTileSize;
            m_worldUploadCommandList->CopyTextureRegion(&dstLoc, dstX, dstY, 0, &srcLoc, nullptr);

            uploadedBytes += uploadBytes;
            transientUploads.push_back(std::move(uploadBuffer));
        }

        D3D12_RESOURCE_BARRIER toShader{};
        toShader.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        toShader.Transition.pResource = m_worldSatelliteAtlasTexture.Get();
        toShader.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        toShader.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        toShader.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        m_worldUploadCommandList->ResourceBarrier(1, &toShader);
    }

    if (!pageTableUpdates.empty()) {
        bool anyValid = false;
        uint32_t minX = kWorldSatellitePageTableWidth;
        uint32_t minY = kWorldSatellitePageTableHeight;
        uint32_t maxX = 0;
        uint32_t maxY = 0;
        for (const WorldPageTableUpdate& u : pageTableUpdates) {
            if (u.x >= kWorldSatellitePageTableWidth || u.y >= kWorldSatellitePageTableHeight) {
                continue;
            }
            anyValid = true;
            minX = std::min(minX, u.x);
            minY = std::min(minY, u.y);
            maxX = std::max(maxX, u.x);
            maxY = std::max(maxY, u.y);
            const size_t idx = static_cast<size_t>(u.y) * static_cast<size_t>(kWorldSatellitePageTableWidth) + static_cast<size_t>(u.x);
            m_worldSatellitePageTableCpu[idx] = u.value;
            const size_t keyIdx = idx * 2u;
            m_worldSatellitePageKeyCpu[keyIdx + 0] = u.key0;
            m_worldSatellitePageKeyCpu[keyIdx + 1] = u.key1;
        }

        if (anyValid) {
            D3D12_RESOURCE_BARRIER toCopy[2]{};
            toCopy[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            toCopy[0].Transition.pResource = m_worldSatellitePageTableTexture.Get();
            toCopy[0].Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            toCopy[0].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
            toCopy[0].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            toCopy[1].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            toCopy[1].Transition.pResource = m_worldSatellitePageKeyTexture.Get();
            toCopy[1].Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            toCopy[1].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
            toCopy[1].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            m_worldUploadCommandList->ResourceBarrier(2, toCopy);

            const uint32_t rectW = (maxX - minX) + 1u;
            const uint32_t rectH = (maxY - minY) + 1u;
            const UINT pageRowBytes = rectW * sizeof(uint32_t);
            const UINT pageRowPitch =
                ((pageRowBytes + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1u) / D3D12_TEXTURE_DATA_PITCH_ALIGNMENT) *
                D3D12_TEXTURE_DATA_PITCH_ALIGNMENT;
            const UINT64 pageUploadBytes = static_cast<UINT64>(pageRowPitch) * static_cast<UINT64>(rectH);

            D3D12_HEAP_PROPERTIES uploadHeap{};
            uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;
            D3D12_RESOURCE_DESC uploadDesc{};
            uploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
            uploadDesc.Width = pageUploadBytes;
            uploadDesc.Height = 1;
            uploadDesc.DepthOrArraySize = 1;
            uploadDesc.MipLevels = 1;
            uploadDesc.SampleDesc.Count = 1;
            uploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

            ComPtr<ID3D12Resource> uploadBuffer;
            HRESULT hr = m_device->CreateCommittedResource(
                &uploadHeap,
                D3D12_HEAP_FLAG_NONE,
                &uploadDesc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(uploadBuffer.ReleaseAndGetAddressOf()));
            if (FAILED(hr)) {
                error = HrMessage("CreateCommittedResource(world page table rect upload) failed", hr);
                m_worldStreamingStats.uploadFailures += 1;
                return false;
            }

            uint8_t* mapped = nullptr;
            D3D12_RANGE readRange{};
            hr = uploadBuffer->Map(0, &readRange, reinterpret_cast<void**>(&mapped));
            if (FAILED(hr) || mapped == nullptr) {
                error = HrMessage("Map(world page table rect upload) failed", hr);
                m_worldStreamingStats.uploadFailures += 1;
                return false;
            }
            for (uint32_t row = 0; row < rectH; ++row) {
                const uint32_t srcY = minY + row;
                const uint32_t* src = m_worldSatellitePageTableCpu.data() +
                    static_cast<size_t>(srcY) * static_cast<size_t>(kWorldSatellitePageTableWidth) + static_cast<size_t>(minX);
                std::memcpy(mapped + static_cast<size_t>(row) * static_cast<size_t>(pageRowPitch), src, static_cast<size_t>(pageRowBytes));
            }
            uploadBuffer->Unmap(0, nullptr);

            D3D12_TEXTURE_COPY_LOCATION dstLoc{};
            dstLoc.pResource = m_worldSatellitePageTableTexture.Get();
            dstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            dstLoc.SubresourceIndex = 0;

            D3D12_TEXTURE_COPY_LOCATION srcLoc{};
            srcLoc.pResource = uploadBuffer.Get();
            srcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            srcLoc.PlacedFootprint.Offset = 0;
            srcLoc.PlacedFootprint.Footprint.Format = DXGI_FORMAT_R32_UINT;
            srcLoc.PlacedFootprint.Footprint.Width = rectW;
            srcLoc.PlacedFootprint.Footprint.Height = rectH;
            srcLoc.PlacedFootprint.Footprint.Depth = 1;
            srcLoc.PlacedFootprint.Footprint.RowPitch = pageRowPitch;

            D3D12_BOX srcBox{};
            srcBox.left = 0;
            srcBox.top = 0;
            srcBox.front = 0;
            srcBox.right = rectW;
            srcBox.bottom = rectH;
            srcBox.back = 1;
            m_worldUploadCommandList->CopyTextureRegion(&dstLoc, minX, minY, 0, &srcLoc, &srcBox);

            uploadedBytes += pageUploadBytes;
            transientUploads.push_back(std::move(uploadBuffer));

            const UINT keyRowBytes = rectW * sizeof(uint32_t) * 2u;
            const UINT keyRowPitch =
                ((keyRowBytes + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1u) / D3D12_TEXTURE_DATA_PITCH_ALIGNMENT) *
                D3D12_TEXTURE_DATA_PITCH_ALIGNMENT;
            const UINT64 keyUploadBytes = static_cast<UINT64>(keyRowPitch) * static_cast<UINT64>(rectH);

            D3D12_RESOURCE_DESC keyUploadDesc{};
            keyUploadDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
            keyUploadDesc.Width = keyUploadBytes;
            keyUploadDesc.Height = 1;
            keyUploadDesc.DepthOrArraySize = 1;
            keyUploadDesc.MipLevels = 1;
            keyUploadDesc.SampleDesc.Count = 1;
            keyUploadDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

            ComPtr<ID3D12Resource> keyUploadBuffer;
            hr = m_device->CreateCommittedResource(
                &uploadHeap,
                D3D12_HEAP_FLAG_NONE,
                &keyUploadDesc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(keyUploadBuffer.ReleaseAndGetAddressOf()));
            if (FAILED(hr)) {
                error = HrMessage("CreateCommittedResource(world page key rect upload) failed", hr);
                m_worldStreamingStats.uploadFailures += 1;
                return false;
            }

            uint8_t* keyMapped = nullptr;
            hr = keyUploadBuffer->Map(0, &readRange, reinterpret_cast<void**>(&keyMapped));
            if (FAILED(hr) || keyMapped == nullptr) {
                error = HrMessage("Map(world page key rect upload) failed", hr);
                m_worldStreamingStats.uploadFailures += 1;
                return false;
            }
            for (uint32_t row = 0; row < rectH; ++row) {
                const uint32_t srcY = minY + row;
                const uint32_t* src = m_worldSatellitePageKeyCpu.data() +
                    ((static_cast<size_t>(srcY) * static_cast<size_t>(kWorldSatellitePageTableWidth) + static_cast<size_t>(minX)) * 2u);
                std::memcpy(keyMapped + static_cast<size_t>(row) * static_cast<size_t>(keyRowPitch), src, static_cast<size_t>(keyRowBytes));
            }
            keyUploadBuffer->Unmap(0, nullptr);

            D3D12_TEXTURE_COPY_LOCATION keyDstLoc{};
            keyDstLoc.pResource = m_worldSatellitePageKeyTexture.Get();
            keyDstLoc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            keyDstLoc.SubresourceIndex = 0;

            D3D12_TEXTURE_COPY_LOCATION keySrcLoc{};
            keySrcLoc.pResource = keyUploadBuffer.Get();
            keySrcLoc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            keySrcLoc.PlacedFootprint.Offset = 0;
            keySrcLoc.PlacedFootprint.Footprint.Format = DXGI_FORMAT_R32G32_UINT;
            keySrcLoc.PlacedFootprint.Footprint.Width = rectW;
            keySrcLoc.PlacedFootprint.Footprint.Height = rectH;
            keySrcLoc.PlacedFootprint.Footprint.Depth = 1;
            keySrcLoc.PlacedFootprint.Footprint.RowPitch = keyRowPitch;

            m_worldUploadCommandList->CopyTextureRegion(&keyDstLoc, minX, minY, 0, &keySrcLoc, &srcBox);

            D3D12_RESOURCE_BARRIER toShader[2]{};
            toShader[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            toShader[0].Transition.pResource = m_worldSatellitePageTableTexture.Get();
            toShader[0].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
            toShader[0].Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            toShader[0].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            toShader[1].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            toShader[1].Transition.pResource = m_worldSatellitePageKeyTexture.Get();
            toShader[1].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
            toShader[1].Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            toShader[1].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            m_worldUploadCommandList->ResourceBarrier(2, toShader);

            uploadedBytes += keyUploadBytes;
            transientUploads.push_back(std::move(keyUploadBuffer));
        }
    }

    if (FAILED(m_worldUploadCommandList->Close())) {
        error = "World upload command list close failed";
        m_worldStreamingStats.uploadFailures += 1;
        return false;
    }

    ID3D12CommandList* lists[] = {m_worldUploadCommandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, lists);
    const UINT64 uploadFence = ++m_fenceValue;
    m_commandQueue->Signal(m_fence.Get(), uploadFence);
    m_worldUploadFenceValue = uploadFence;

    for (auto& res : transientUploads) {
        DeferredResource d{};
        d.resource = std::move(res);
        d.safeFenceValue = uploadFence;
        m_retiredResources.push_back(std::move(d));
    }

    m_worldStreamingStats.resourcesReady = true;
    m_worldStreamingStats.uploadBatches += 1;
    m_worldStreamingStats.atlasPageUploads += pageUploads.size();
    m_worldStreamingStats.pageTableWrites += pageTableUpdates.size();
    m_worldStreamingStats.uploadBytes += uploadedBytes;
    return true;
}

void D3D12Renderer::WaitForFrame(UINT frameIndex) {
    if (m_fence->GetCompletedValue() < m_frames[frameIndex].fenceValue) {
        m_fence->SetEventOnCompletion(m_frames[frameIndex].fenceValue, m_fenceEvent);
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }
}

void D3D12Renderer::CleanupDeferredTerrainResources() {
    if (!m_fence) {
        return;
    }
    const UINT64 completed = m_fence->GetCompletedValue();

    if (m_terrainUploadFenceValue != 0 && completed >= m_terrainUploadFenceValue) {
        m_terrainMesh.ReleaseUploadBuffers();
        m_terrainUploadFenceValue = 0;
    }
    if (m_satelliteUploadFenceValue != 0 && completed >= m_satelliteUploadFenceValue) {
        for (auto& upload : m_satelliteLodUploads) {
            upload.Reset();
        }
        m_satelliteUploadFenceValue = 0;
    }
    if (m_worldUploadFenceValue != 0 && completed >= m_worldUploadFenceValue) {
        m_worldUploadFenceValue = 0;
    }

    auto it = m_retiredTerrainMeshes.begin();
    while (it != m_retiredTerrainMeshes.end()) {
        if (completed >= it->safeFenceValue) {
            it = m_retiredTerrainMeshes.erase(it);
        } else {
            ++it;
        }
    }

    auto r = m_retiredResources.begin();
    while (r != m_retiredResources.end()) {
        if (completed >= r->safeFenceValue) {
            r = m_retiredResources.erase(r);
        } else {
            ++r;
        }
    }
}

void D3D12Renderer::CreateRenderTargetViews() {
    D3D12_CPU_DESCRIPTOR_HANDLE handle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
    for (UINT i = 0; i < kFrameCount; ++i) {
        m_swapChain->GetBuffer(i, IID_PPV_ARGS(m_renderTargets[i].ReleaseAndGetAddressOf()));
        m_device->CreateRenderTargetView(m_renderTargets[i].Get(), nullptr, handle);
        handle.ptr += m_rtvDescriptorSize;
    }
}

void D3D12Renderer::ReleaseRenderTargets() {
    for (auto& rt : m_renderTargets) {
        rt.Reset();
    }
}

D3D12_CPU_DESCRIPTOR_HANDLE D3D12Renderer::CpuSrv(UINT index) const {
    D3D12_CPU_DESCRIPTOR_HANDLE handle = m_srvHeap->GetCPUDescriptorHandleForHeapStart();
    handle.ptr += static_cast<SIZE_T>(index) * m_srvDescriptorSize;
    return handle;
}

D3D12_GPU_DESCRIPTOR_HANDLE D3D12Renderer::GpuSrv(UINT index) const {
    D3D12_GPU_DESCRIPTOR_HANDLE handle = m_srvHeap->GetGPUDescriptorHandleForHeapStart();
    handle.ptr += static_cast<UINT64>(index) * static_cast<UINT64>(m_srvDescriptorSize);
    return handle;
}

} // namespace flight
