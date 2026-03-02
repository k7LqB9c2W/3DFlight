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
    bool SetTerrainMesh(const MeshData& mesh, const Double3& anchorEcef, std::string& error);
    void SetRenderOldEarthSphere(bool enabled) { m_renderOldEarthSphere = enabled; }
    [[nodiscard]] bool IsRenderOldEarthSphereEnabled() const { return m_renderOldEarthSphere; }
    [[nodiscard]] bool HasLandmask() const { return m_hasLandmaskTexture; }

private:
    static constexpr UINT kFrameCount = 2;
    static constexpr UINT kFontSrvIndex = 0;
    static constexpr UINT kLandmaskSrvIndex = 1;
    static constexpr UINT kSkyboxSrvIndex = 2;

    struct FrameContext {
        Microsoft::WRL::ComPtr<ID3D12CommandAllocator> allocator;
        UINT64 fenceValue = 0;
    };

    struct alignas(256) SceneConstants {
        DirectX::XMFLOAT4X4 viewProj{};
        DirectX::XMFLOAT4 earthCenterRadius{};
        DirectX::XMFLOAT4 sunDirIntensity{};
        DirectX::XMFLOAT4 atmosphereParams{};
    };

    struct alignas(256) ObjectConstants {
        DirectX::XMFLOAT4X4 model{};
        DirectX::XMFLOAT4 colorAndFlags{};
    };

    bool CreateDeviceResources(std::string& error);
    bool CreateSwapchainResources(std::string& error);
    bool CreateDepthBuffer(std::string& error);
    bool CreatePipeline(std::string& error);
    bool CreateConstantBuffers(std::string& error);
    bool CreateLandmaskTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
    bool CreateSkyboxTexture(ID3D12GraphicsCommandList* commandList, std::string& error);
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
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_planePso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_earthPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_skyboxPso;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_terrainPso;

    GpuMesh m_earthMesh;
    GpuMesh m_planeMesh;
    GpuMesh m_skyboxMesh;
    GpuMesh m_terrainMesh;
    Double3 m_terrainAnchorEcef{};
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

    Microsoft::WRL::ComPtr<IWICImagingFactory> m_wicFactory;

    D3D12_VIEWPORT m_viewport{};
    D3D12_RECT m_scissor{};

    bool m_imguiInitialized = false;
};

} // namespace flight
