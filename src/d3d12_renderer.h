#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <windows.h>

#include <d3d12.h>
#include <dxgi1_6.h>
#include <wincodec.h>
#include <wrl/client.h>
#include <DirectXMath.h>

#include "mesh.h"
#include "sim.h"

struct ImDrawData;

namespace flight {

class D3D12Renderer {
public:
    struct SatelliteLodTexture {
        const uint8_t* rgbaPixels = nullptr;
        uint32_t width = 0;
        uint32_t height = 0;
        // Unwrapped lon/lat bounds: x=lonW, y=lonE, z=latS, w=latN.
        DirectX::XMFLOAT4 boundsLonLat{0.0f, 0.0f, 0.0f, 0.0f};
        bool valid = false;
    };

    struct WorldAtlasPageUpload {
        const uint8_t* rgbaPixels = nullptr;
        uint32_t atlasPageX = 0;
        uint32_t atlasPageY = 0;
    };

    struct WorldPageTableUpdate {
        uint32_t x = 0;
        uint32_t y = 0;
        uint32_t key0 = 0;
        uint32_t key1 = 0;
        uint32_t value = 0;
    };

    struct WorldStreamingStats {
        bool resourcesReady = false;
        uint64_t uploadBatches = 0;
        uint64_t atlasPageUploads = 0;
        uint64_t pageTableWrites = 0;
        uint64_t uploadBytes = 0;
        uint64_t uploadFailures = 0;
    };

    struct TerrainVisualSettings {
        float colorHeightMaxMeters = 6000.0f;
        float lodTransitionWidthMeters = 5000.0f;
        float midRingMultiplier = 2.6f;
        float farRingMultiplier = 2.6f;
        float hazeStrength = 0.0f;
        float hazeAltitudeRangeMeters = 25000.0f;
        float colorContrast = 1.0f;
        float slopeShadingStrength = 0.35f;
        float specularStrength = 0.14f;
        float lodSeamBlendStrength = 0.20f;
        bool satelliteEnabled = true;
        float satelliteBlend = 1.0f;
    };

    bool Initialize(
        HWND hwnd,
        uint32_t width,
        uint32_t height,
        const std::filesystem::path& shaderDir,
        const std::filesystem::path& assetDir,
        std::string& error);

    void Shutdown();
    void WaitForGpu();
    void Resize(uint32_t width, uint32_t height);

    void BeginImGuiFrame();
    void Render(const FlightSim& sim, ImDrawData* imguiDrawData);

    bool SetPlaneMesh(const MeshData& mesh, std::string& error);
    bool SetPlaneTexture(const uint8_t* rgbaPixels, uint32_t width, uint32_t height, std::string& error);
    bool SetSatelliteLodTextures(const std::array<SatelliteLodTexture, 3>& lods, std::string& error);
    bool UploadWorldLockedSatelliteData(
        const std::vector<WorldAtlasPageUpload>& pageUploads,
        const std::vector<WorldPageTableUpdate>& pageTableUpdates,
        std::string& error);
    bool SetTerrainMesh(const MeshData& mesh, const Double3& anchorEcef, const DirectX::XMFLOAT4& renderParams, std::string& error);
    void SetTerrainVisualSettings(const TerrainVisualSettings& settings);
    [[nodiscard]] const TerrainVisualSettings& GetTerrainVisualSettings() const { return m_terrainVisualSettings; }
    void SetAerialPerspectiveDepthMeters(float depthMeters);
    [[nodiscard]] float AerialPerspectiveDepthMeters() const { return m_aerialPerspectiveDepthMeters; }
    void SetSunDirection(const Double3& dir);
    [[nodiscard]] Double3 SunDirection() const { return m_sunDirection; }
    void SetRenderOldEarthSphere(bool enabled) { m_renderOldEarthSphere = enabled; }
    [[nodiscard]] bool IsRenderOldEarthSphereEnabled() const { return m_renderOldEarthSphere; }
    [[nodiscard]] bool HasLandmask() const { return m_hasLandmaskTexture; }
    void SetAtmosphereEnabled(bool enabled) { m_atmosphereEnabled = enabled; }
    [[nodiscard]] bool IsAtmosphereEnabled() const { return m_atmosphereEnabled; }
    void SetMultipleScatteringEnabled(bool enabled) { m_multipleScatteringEnabled = enabled; }
    [[nodiscard]] bool IsMultipleScatteringEnabled() const { return m_multipleScatteringEnabled; }
    void SetAtmosphereExposure(float exposure) { m_atmosphereExposure = exposure; }
    [[nodiscard]] float AtmosphereExposure() const { return m_atmosphereExposure; }
    void SetCameraZoomRangeMeters(float minDistance, float maxDistance);
    void AddCameraZoomSteps(float wheelSteps);
    [[nodiscard]] float CameraFollowDistanceMeters() const { return m_cameraFollowDistanceMeters; }
    [[nodiscard]] float CameraZoomMinDistanceMeters() const { return m_cameraMinFollowDistanceMeters; }
    [[nodiscard]] float CameraZoomMaxDistanceMeters() const { return m_cameraMaxFollowDistanceMeters; }
    [[nodiscard]] bool HasSatelliteSourceFile() const { return m_hasEarthAlbedoSourceFile; }
    [[nodiscard]] const std::filesystem::path& SatelliteSourcePath() const { return m_earthAlbedoSourcePath; }
    [[nodiscard]] DirectX::XMFLOAT4 SatelliteLonLatBounds() const { return m_earthAlbedoBoundsLonLat; }
    [[nodiscard]] bool SatelliteWrapsLongitude() const { return m_earthAlbedoWrapLon > 0.5f; }
    void SetWorldLockedSatelliteEnabled(bool enabled) { m_worldLockedSatelliteEnabled = enabled; }
    [[nodiscard]] bool IsWorldLockedSatelliteEnabled() const { return m_worldLockedSatelliteEnabled; }
    void SetWorldSamplingZooms(int nearZoom, int midZoom, int farZoom) {
        m_worldSamplingZooms = {nearZoom, midZoom, farZoom};
    }
    [[nodiscard]] WorldStreamingStats GetWorldStreamingStats() const { return m_worldStreamingStats; }

private:
    static constexpr UINT kFrameCount = 2;
    static constexpr UINT kFontSrvIndex = 0;
    static constexpr UINT kSrvTableStartIndex = 1;
    static constexpr UINT kLandmaskSrvIndex = 1;
    static constexpr UINT kSkyboxSrvIndex = 2;
    static constexpr UINT kTransmittanceSrvIndex = 3;
    static constexpr UINT kSkyViewSrvIndex = 4;
    static constexpr UINT kMultipleScatteringSrvIndex = 5;
    static constexpr UINT kAerialPerspectiveSrvIndex = 6;
    static constexpr UINT kEarthAlbedoSrvIndex = 7;
    // Descriptor index maps to t-register as (index - kSrvTableStartIndex), because slot 0 is reserved for ImGui font.
    // With kSrvTableStartIndex=1:
    //   t8  -> descriptor 9
    //   t9  -> descriptor 10
    //   t10 -> descriptor 11
    //   t11 -> descriptor 12
    //   t12 -> descriptor 13
    //   t13 -> descriptor 14
    //   t14 -> descriptor 15
    //   t19 -> descriptor 20 (world atlas)
    //   t20 -> descriptor 21 (world page table values)
    //   t21 -> descriptor 22 (world page table keys)
    static constexpr UINT kSatelliteNearSrvIndex = 9;
    static constexpr UINT kSatelliteMidSrvIndex = 10;
    static constexpr UINT kSatelliteFarSrvIndex = 11;
    static constexpr UINT kSatellitePrevNearSrvIndex = 12;
    static constexpr UINT kSatellitePrevMidSrvIndex = 13;
    static constexpr UINT kSatellitePrevFarSrvIndex = 14;
    static constexpr UINT kModelAlbedoSrvIndex = 15;
    static constexpr UINT kWorldSatelliteAtlasSrvIndex = 20;
    static constexpr UINT kWorldSatellitePageTableSrvIndex = 21;
    static constexpr UINT kWorldSatellitePageKeySrvIndex = 22;
    static constexpr UINT kUavTableStartIndex = 16;
    static constexpr UINT kTransmittanceUavIndex = kUavTableStartIndex + 0;
    static constexpr UINT kSkyViewUavIndex = kUavTableStartIndex + 1;
    static constexpr UINT kMultipleScatteringUavIndex = kUavTableStartIndex + 2;
    static constexpr UINT kAerialPerspectiveUavIndex = kUavTableStartIndex + 3;
    static constexpr uint32_t kWorldSatelliteTileSize = 256;
    static constexpr uint32_t kWorldSatelliteAtlasPagesX = 16;
    static constexpr uint32_t kWorldSatelliteAtlasPagesY = 16;
    static constexpr uint32_t kWorldSatellitePageTableWidth = 1024;
    static constexpr uint32_t kWorldSatellitePageTableHeight = 1024;

    struct FrameContext {
        Microsoft::WRL::ComPtr<ID3D12CommandAllocator> allocator;
        UINT64 fenceValue = 0;
    };

    struct AtmosphereSettings {
        // EGSR 2020 Table 1 coefficients, converted from x 1e-6 m^-1 to m^-1.
        DirectX::XMFLOAT3 rayleighScattering = {5.802e-6f, 13.558e-6f, 33.1e-6f};
        DirectX::XMFLOAT3 rayleighAbsorption = {0.0f, 0.0f, 0.0f};
        DirectX::XMFLOAT3 mieScattering = {3.996e-6f, 3.996e-6f, 3.996e-6f};
        // Use absorption (not extinction) here because extinction is computed as sigma_s + sigma_a.
        // Table extinction often cited as 4.40e-6, so absorption ~= 4.40e-6 - 3.996e-6 = 0.404e-6.
        DirectX::XMFLOAT3 mieAbsorption = {0.404e-6f, 0.404e-6f, 0.404e-6f};
        DirectX::XMFLOAT3 ozoneAbsorption = {0.650e-6f, 1.881e-6f, 0.085e-6f};
        float rayleighScaleHeightMeters = 8000.0f;
        float mieScaleHeightMeters = 1200.0f;
        float ozoneCenterHeightMeters = 25000.0f;
        float ozoneHalfWidthMeters = 15000.0f;
        float atmosphereHeightMeters = 100000.0f;
        float miePhaseG = 0.8f;
        float sunAngularRadiusRad = 0.004675f; // ~0.267 deg
        float sunIlluminance = 20.0f;
    };

    struct alignas(256) SceneConstants {
        DirectX::XMFLOAT4X4 viewProj{};
        DirectX::XMFLOAT4X4 invViewProj{};
        DirectX::XMFLOAT4 earthCenterRadius{};
        DirectX::XMFLOAT4 sunDirIntensity{};
        DirectX::XMFLOAT4 cameraPosTopRadius{};
        DirectX::XMFLOAT4 cameraUpAndTime{};
        DirectX::XMFLOAT4 viewportAndAerialDepth{};
        DirectX::XMFLOAT4 atmosphereFlags{};
        DirectX::XMFLOAT4 rayleighScatteringAndScale{};
        DirectX::XMFLOAT4 rayleighAbsorptionPad{};
        DirectX::XMFLOAT4 mieScatteringAndScale{};
        DirectX::XMFLOAT4 mieAbsorptionAndG{};
        DirectX::XMFLOAT4 ozoneAbsorptionAndCenter{};
        DirectX::XMFLOAT4 ozoneHalfWidthAtmosphereHeightSunRadius{};
    };

    struct alignas(256) ObjectConstants {
        DirectX::XMFLOAT4X4 model{};
        DirectX::XMFLOAT4 colorAndFlags{};
        DirectX::XMFLOAT4 tuning0{};
        DirectX::XMFLOAT4 tuning1{};
        DirectX::XMFLOAT4 tuning2{};
        DirectX::XMFLOAT4 tuning3{};
        DirectX::XMFLOAT4 tuning4{};
        DirectX::XMFLOAT4 tuning5{};
        DirectX::XMFLOAT4 tuning6{};
        DirectX::XMFLOAT4 tuning7{};
        DirectX::XMFLOAT4 tuning8{};
        DirectX::XMFLOAT4 tuning9{};
        DirectX::XMFLOAT4 tuning10{};
        DirectX::XMFLOAT4 tuning11{};
        DirectX::XMFLOAT4 tuning12{};
        DirectX::XMFLOAT4 tuning13{};
        DirectX::XMFLOAT4 tuning14{};
    };

    bool CreateDeviceResources(std::string& error);
    bool CreateSwapchainResources(std::string& error);
    bool CreateDepthBuffer(std::string& error);
    bool CreatePipeline(std::string& error);
    bool CreateConstantBuffers(std::string& error);
    bool CreateAtmosphereResources(std::string& error);
    bool CreateWorldStreamingResources(ID3D12GraphicsCommandList* commandList, std::string& error);
    void DispatchAtmosphereLuts(ID3D12GraphicsCommandList* commandList, const SceneConstants& sceneCb);
    bool CreateLandmaskTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
    bool CreateSkyboxTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
    bool CreateEarthAlbedoTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
    bool CreatePlaneTextureFromPixels(
        ID3D12GraphicsCommandList* commandList,
        const uint8_t* rgbaPixels,
        uint32_t width,
        uint32_t height,
        std::string& error);
    bool CreateSatelliteTextureFromPixels(
        ID3D12GraphicsCommandList* commandList,
        UINT srvIndex,
        Microsoft::WRL::ComPtr<ID3D12Resource>& outTexture,
        Microsoft::WRL::ComPtr<ID3D12Resource>& outUpload,
        const uint8_t* rgbaPixels,
        uint32_t width,
        uint32_t height,
        std::string& error);
    bool CreateLandmaskTextureFromPixels(
        ID3D12GraphicsCommandList* commandList,
        const uint8_t* pixels,
        uint32_t width,
        uint32_t height,
        bool markLandmaskPresent,
        std::string& error);
    bool LoadLandmaskPixels(
        const std::filesystem::path& imagePath,
        std::vector<uint8_t>& pixels,
        uint32_t& outWidth,
        uint32_t& outHeight,
        std::string& error);
    bool LoadColorPixels(
        const std::filesystem::path& imagePath,
        std::vector<uint8_t>& pixelsRgba,
        uint32_t& outWidth,
        uint32_t& outHeight,
        uint32_t maxDim,
        std::string& error);
    bool CreateSkyboxTextureFromFaces(
        ID3D12GraphicsCommandList* commandList,
        const std::array<std::vector<uint8_t>, 6>& facePixels,
        uint32_t width,
        uint32_t height,
        std::string& error);
    void CreateSatelliteSrvForResource(ID3D12Resource* texture, UINT srvIndex);

    bool CompileShader(
        const std::filesystem::path& path,
        const char* entry,
        const char* target,
        Microsoft::WRL::ComPtr<ID3DBlob>& outBlob,
        std::string& error);

    void WaitForFrame(UINT frameIndex);
    void WaitForFenceValue(UINT64 fenceValue);
    void CleanupDeferredTerrainResources();
    void CreateRenderTargetViews();
    void ReleaseRenderTargets();

    D3D12_CPU_DESCRIPTOR_HANDLE CpuSrv(UINT index) const;
    D3D12_GPU_DESCRIPTOR_HANDLE GpuSrv(UINT index) const;

    HWND m_hwnd = nullptr;
    uint32_t m_width = 1;
    uint32_t m_height = 1;
    std::filesystem::path m_shaderDir;
    std::filesystem::path m_assetDir;

    Microsoft::WRL::ComPtr<IDXGIFactory4> m_factory;
    Microsoft::WRL::ComPtr<ID3D12Device> m_device;
    Microsoft::WRL::ComPtr<ID3D12CommandQueue> m_commandQueue;
    Microsoft::WRL::ComPtr<IDXGISwapChain3> m_swapChain;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> m_commandList;
    Microsoft::WRL::ComPtr<ID3D12CommandAllocator> m_uploadAllocator;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> m_uploadCommandList;
    Microsoft::WRL::ComPtr<ID3D12CommandAllocator> m_satelliteUploadAllocator;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> m_satelliteUploadCommandList;
    Microsoft::WRL::ComPtr<ID3D12CommandAllocator> m_worldUploadAllocator;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> m_worldUploadCommandList;

    std::array<FrameContext, kFrameCount> m_frames;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, kFrameCount> m_renderTargets;
    UINT m_frameIndex = 0;

    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_dsvHeap;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srvHeap;
    UINT m_rtvDescriptorSize = 0;
    UINT m_srvDescriptorSize = 0;

    Microsoft::WRL::ComPtr<ID3D12Resource> m_depthBuffer;

    Microsoft::WRL::ComPtr<ID3D12Fence> m_fence;
    UINT64 m_fenceValue = 0;
    HANDLE m_fenceEvent = nullptr;

    Microsoft::WRL::ComPtr<ID3D12RootSignature> m_rootSignature;
    Microsoft::WRL::ComPtr<ID3D12RootSignature> m_computeRootSignature;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_planePso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_earthPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_skyboxPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_terrainPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_terrainBlendPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_transmittanceLutPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_skyViewLutPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_multiScatteringLutPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_aerialPerspectiveLutPso;

    GpuMesh m_earthMesh;
    GpuMesh m_planeMesh;
    GpuMesh m_skyboxMesh;
    GpuMesh m_terrainMesh;
    GpuMesh m_prevTerrainMesh;
    struct DeferredTerrainMesh {
        GpuMesh mesh;
        UINT64 safeFenceValue = 0;
    };
    struct DeferredResource {
        Microsoft::WRL::ComPtr<ID3D12Resource> resource;
        UINT64 safeFenceValue = 0;
    };
    std::vector<DeferredTerrainMesh> m_retiredTerrainMeshes;
    std::vector<DeferredResource> m_retiredResources;
    UINT64 m_terrainUploadFenceValue = 0;
    UINT64 m_satelliteUploadFenceValue = 0;
    UINT64 m_worldUploadFenceValue = 0;
    Double3 m_terrainAnchorEcef{};
    Double3 m_prevTerrainAnchorEcef{};
    DirectX::XMFLOAT4 m_terrainRenderParams{40000.0f, 6000.0f, 5000.0f, 900000.0f};
    DirectX::XMFLOAT4 m_prevTerrainRenderParams{40000.0f, 6000.0f, 5000.0f, 900000.0f};
    TerrainVisualSettings m_terrainVisualSettings{};
    bool m_hasTerrainMesh = false;
    bool m_hasPrevTerrainMesh = false;
    bool m_terrainTransitionActive = false;
    float m_terrainTransitionT = 1.0f;
    float m_terrainTransitionDurationSeconds = 0.55f;
    std::chrono::steady_clock::time_point m_terrainTransitionStart{};
    bool m_renderOldEarthSphere = true;

    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, kFrameCount> m_sceneCb;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, kFrameCount> m_objectCb;
    std::array<uint8_t*, kFrameCount> m_sceneCbMapped{};
    std::array<uint8_t*, kFrameCount> m_objectCbMapped{};
    UINT64 m_sceneCbSize = 0;
    UINT64 m_objectCbStride = 0;

    Microsoft::WRL::ComPtr<ID3D12Resource> m_landmaskTexture;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_landmaskUpload;
    bool m_hasLandmaskTexture = false;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_skyboxTexture;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_skyboxUpload;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_earthAlbedoTexture;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_earthAlbedoUpload;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 3> m_satelliteLodTextures;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 3> m_satelliteLodUploads;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 3> m_satellitePrevLodTextures;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 3> m_satellitePrevLodUploads;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_modelAlbedoTexture;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_modelAlbedoUpload;
    bool m_hasModelAlbedoTexture = false;
    std::array<DirectX::XMFLOAT4, 3> m_satelliteLodBounds{
        DirectX::XMFLOAT4{0.0f, 0.0f, 0.0f, 0.0f},
        DirectX::XMFLOAT4{0.0f, 0.0f, 0.0f, 0.0f},
        DirectX::XMFLOAT4{0.0f, 0.0f, 0.0f, 0.0f}};
    std::array<float, 3> m_satelliteLodValid{0.0f, 0.0f, 0.0f};
    std::array<DirectX::XMFLOAT4, 3> m_satellitePrevLodBounds{
        DirectX::XMFLOAT4{0.0f, 0.0f, 0.0f, 0.0f},
        DirectX::XMFLOAT4{0.0f, 0.0f, 0.0f, 0.0f},
        DirectX::XMFLOAT4{0.0f, 0.0f, 0.0f, 0.0f}};
    std::array<float, 3> m_satellitePrevLodValid{0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point m_satelliteTransitionStart{};
    float m_satelliteTransitionDurationSeconds = 0.22f;
    float m_satelliteTransitionT = 1.0f;
    bool m_satelliteTransitionActive = false;
    std::filesystem::path m_earthAlbedoSourcePath;
    bool m_hasEarthAlbedoSourceFile = false;
    DirectX::XMFLOAT4 m_earthAlbedoBoundsLonLat = {-180.0f, 180.0f, -90.0f, 90.0f}; // lonW, lonE, latS, latN
    float m_earthAlbedoWrapLon = 1.0f;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_worldSatelliteAtlasTexture;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_worldSatellitePageTableTexture;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_worldSatellitePageKeyTexture;
    std::vector<uint32_t> m_worldSatellitePageTableCpu;
    std::vector<uint32_t> m_worldSatellitePageKeyCpu;
    bool m_worldStreamingResourcesReady = false;
    bool m_worldLockedSatelliteEnabled = false;
    std::array<int, 3> m_worldSamplingZooms{13, 12, 10};
    WorldStreamingStats m_worldStreamingStats{};
    Microsoft::WRL::ComPtr<ID3D12Resource> m_transmittanceLut;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_skyViewLut;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_multipleScatteringLut;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_aerialPerspectiveLut;

    Microsoft::WRL::ComPtr<IWICImagingFactory> m_wicFactory;

    D3D12_VIEWPORT m_viewport{};
    D3D12_RECT m_scissor{};
    float m_cameraFollowDistanceMeters = 250.0f;
    float m_cameraMinFollowDistanceMeters = 45.0f;
    float m_cameraMaxFollowDistanceMeters = 3000.0f;

    AtmosphereSettings m_atmosphereSettings{};
    Double3 m_sunDirection{0.35, 0.82, 0.45};
    bool m_atmosphereEnabled = true;
    bool m_multipleScatteringEnabled = true;
    float m_atmosphereExposure = 1.0f;
    float m_aerialPerspectiveDepthMeters = 180000.0f;

    bool m_imguiInitialized = false;
};

} // namespace flight
