#pragma once

#include <array>
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
    void AddCameraZoomSteps(float wheelSteps);
    [[nodiscard]] float CameraFollowDistanceMeters() const { return m_cameraFollowDistanceMeters; }
    [[nodiscard]] bool HasSatelliteSourceFile() const { return m_hasEarthAlbedoSourceFile; }
    [[nodiscard]] const std::filesystem::path& SatelliteSourcePath() const { return m_earthAlbedoSourcePath; }
    [[nodiscard]] DirectX::XMFLOAT4 SatelliteLonLatBounds() const { return m_earthAlbedoBoundsLonLat; }
    [[nodiscard]] bool SatelliteWrapsLongitude() const { return m_earthAlbedoWrapLon > 0.5f; }

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
    static constexpr UINT kUavTableStartIndex = 16;
    static constexpr UINT kTransmittanceUavIndex = kUavTableStartIndex + 0;
    static constexpr UINT kSkyViewUavIndex = kUavTableStartIndex + 1;
    static constexpr UINT kMultipleScatteringUavIndex = kUavTableStartIndex + 2;
    static constexpr UINT kAerialPerspectiveUavIndex = kUavTableStartIndex + 3;

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
    };

    bool CreateDeviceResources(std::string& error);
    bool CreateSwapchainResources(std::string& error);
    bool CreateDepthBuffer(std::string& error);
    bool CreatePipeline(std::string& error);
    bool CreateConstantBuffers(std::string& error);
    bool CreateAtmosphereResources(std::string& error);
    void DispatchAtmosphereLuts(ID3D12GraphicsCommandList* commandList, const SceneConstants& sceneCb);
    bool CreateLandmaskTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
    bool CreateSkyboxTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
    bool CreateEarthAlbedoTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
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

    bool CompileShader(
        const std::filesystem::path& path,
        const char* entry,
        const char* target,
        Microsoft::WRL::ComPtr<ID3DBlob>& outBlob,
        std::string& error);

    void WaitForFrame(UINT frameIndex);
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
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_transmittanceLutPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_skyViewLutPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_multiScatteringLutPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_aerialPerspectiveLutPso;

    GpuMesh m_earthMesh;
    GpuMesh m_planeMesh;
    GpuMesh m_skyboxMesh;
    GpuMesh m_terrainMesh;
    struct DeferredTerrainMesh {
        GpuMesh mesh;
        UINT64 safeFenceValue = 0;
    };
    std::vector<DeferredTerrainMesh> m_retiredTerrainMeshes;
    UINT64 m_terrainUploadFenceValue = 0;
    Double3 m_terrainAnchorEcef{};
    DirectX::XMFLOAT4 m_terrainRenderParams{40000.0f, 6000.0f, 5000.0f, 900000.0f};
    TerrainVisualSettings m_terrainVisualSettings{};
    bool m_hasTerrainMesh = false;
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
    std::filesystem::path m_earthAlbedoSourcePath;
    bool m_hasEarthAlbedoSourceFile = false;
    DirectX::XMFLOAT4 m_earthAlbedoBoundsLonLat = {-180.0f, 180.0f, -90.0f, 90.0f}; // lonW, lonE, latS, latN
    float m_earthAlbedoWrapLon = 1.0f;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_transmittanceLut;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_skyViewLut;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_multipleScatteringLut;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_aerialPerspectiveLut;

    Microsoft::WRL::ComPtr<IWICImagingFactory> m_wicFactory;

    D3D12_VIEWPORT m_viewport{};
    D3D12_RECT m_scissor{};
    float m_cameraFollowDistanceMeters = 250.0f;

    AtmosphereSettings m_atmosphereSettings{};
    Double3 m_sunDirection{0.35, 0.82, 0.45};
    bool m_atmosphereEnabled = true;
    bool m_multipleScatteringEnabled = true;
    float m_atmosphereExposure = 1.0f;
    float m_aerialPerspectiveDepthMeters = 180000.0f;

    bool m_imguiInitialized = false;
};

} // namespace flight
