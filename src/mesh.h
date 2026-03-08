#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <d3d12.h>
#include <wrl/client.h>
#include <DirectXMath.h>

namespace flight {

struct Vertex {
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT3 normal;
    DirectX::XMFLOAT2 uv;
    DirectX::XMFLOAT4 color{1.0f, 1.0f, 1.0f, 1.0f};
};

struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    [[nodiscard]] bool IsValid() const {
        return !vertices.empty() && !indices.empty();
    }
};

class GpuMesh {
public:
    bool Upload(ID3D12Device* device, ID3D12GraphicsCommandList* commandList, const MeshData& mesh, std::string& error);
    void Draw(ID3D12GraphicsCommandList* commandList) const;
    void ReleaseUploadBuffers();
    [[nodiscard]] bool IsValid() const { return m_indexCount > 0; }

private:
    Microsoft::WRL::ComPtr<ID3D12Resource> m_vertexBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_indexBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_vertexUpload;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_indexUpload;
    D3D12_VERTEX_BUFFER_VIEW m_vbv{};
    D3D12_INDEX_BUFFER_VIEW m_ibv{};
    uint32_t m_indexCount = 0;
};

MeshData CreatePlaceholderPlane();

} // namespace flight
