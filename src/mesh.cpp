#include "mesh.h"

#include <algorithm>
#include <cstring>

namespace flight {
namespace {

constexpr UINT64 Align(UINT64 value, UINT64 alignment) {
    return (value + alignment - 1) & ~(alignment - 1);
}

bool CreateBuffer(
    ID3D12Device* device,
    UINT64 size,
    D3D12_HEAP_TYPE heapType,
    D3D12_RESOURCE_STATES initialState,
    Microsoft::WRL::ComPtr<ID3D12Resource>& outBuffer,
    std::string& error) {
    D3D12_HEAP_PROPERTIES heapProps{};
    heapProps.Type = heapType;

    D3D12_RESOURCE_DESC desc{};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Width = Align(size, D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT);
    desc.Height = 1;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    const HRESULT hr = device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &desc,
        initialState,
        nullptr,
        IID_PPV_ARGS(outBuffer.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
        error = "CreateCommittedResource(buffer) failed";
        return false;
    }
    return true;
}

} // namespace

bool GpuMesh::Upload(
    ID3D12Device* device,
    ID3D12GraphicsCommandList* commandList,
    const MeshData& mesh,
    std::string& error) {
    if (!mesh.IsValid()) {
        error = "Mesh is empty";
        return false;
    }

    const UINT64 vertexBytes = static_cast<UINT64>(mesh.vertices.size() * sizeof(Vertex));
    const UINT64 indexBytes = static_cast<UINT64>(mesh.indices.size() * sizeof(uint32_t));

    if (!CreateBuffer(device, vertexBytes, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COPY_DEST, m_vertexBuffer, error)) {
        return false;
    }
    if (!CreateBuffer(device, vertexBytes, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ, m_vertexUpload, error)) {
        return false;
    }

    if (!CreateBuffer(device, indexBytes, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COPY_DEST, m_indexBuffer, error)) {
        return false;
    }
    if (!CreateBuffer(device, indexBytes, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ, m_indexUpload, error)) {
        return false;
    }

    void* mapped = nullptr;
    D3D12_RANGE readRange{};
    if (FAILED(m_vertexUpload->Map(0, &readRange, &mapped))) {
        error = "Map(vertex upload) failed";
        return false;
    }
    std::memcpy(mapped, mesh.vertices.data(), static_cast<size_t>(vertexBytes));
    m_vertexUpload->Unmap(0, nullptr);

    if (FAILED(m_indexUpload->Map(0, &readRange, &mapped))) {
        error = "Map(index upload) failed";
        return false;
    }
    std::memcpy(mapped, mesh.indices.data(), static_cast<size_t>(indexBytes));
    m_indexUpload->Unmap(0, nullptr);

    commandList->CopyBufferRegion(m_vertexBuffer.Get(), 0, m_vertexUpload.Get(), 0, vertexBytes);
    commandList->CopyBufferRegion(m_indexBuffer.Get(), 0, m_indexUpload.Get(), 0, indexBytes);

    D3D12_RESOURCE_BARRIER barriers[2]{};
    barriers[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barriers[0].Transition.pResource = m_vertexBuffer.Get();
    barriers[0].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barriers[0].Transition.StateAfter = D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;
    barriers[0].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

    barriers[1].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barriers[1].Transition.pResource = m_indexBuffer.Get();
    barriers[1].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    barriers[1].Transition.StateAfter = D3D12_RESOURCE_STATE_INDEX_BUFFER;
    barriers[1].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

    commandList->ResourceBarrier(2, barriers);

    m_vbv.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
    m_vbv.StrideInBytes = sizeof(Vertex);
    m_vbv.SizeInBytes = static_cast<UINT>(vertexBytes);

    m_ibv.BufferLocation = m_indexBuffer->GetGPUVirtualAddress();
    m_ibv.Format = DXGI_FORMAT_R32_UINT;
    m_ibv.SizeInBytes = static_cast<UINT>(indexBytes);

    m_indexCount = static_cast<uint32_t>(mesh.indices.size());
    return true;
}

void GpuMesh::Draw(ID3D12GraphicsCommandList* commandList) const {
    if (!IsValid()) {
        return;
    }
    commandList->IASetVertexBuffers(0, 1, &m_vbv);
    commandList->IASetIndexBuffer(&m_ibv);
    commandList->DrawIndexedInstanced(m_indexCount, 1, 0, 0, 0);
}

void GpuMesh::ReleaseUploadBuffers() {
    m_vertexUpload.Reset();
    m_indexUpload.Reset();
}

MeshData CreatePlaceholderPlane() {
    MeshData mesh;

    // A tiny low-poly aircraft-like mesh aligned with +Z forward, +Y up.
    const DirectX::XMFLOAT3 nose{0.0f, 0.0f, 30.0f};
    const DirectX::XMFLOAT3 tail{0.0f, 0.0f, -30.0f};
    const DirectX::XMFLOAT3 wingL{-20.0f, 0.0f, 0.0f};
    const DirectX::XMFLOAT3 wingR{20.0f, 0.0f, 0.0f};
    const DirectX::XMFLOAT3 top{0.0f, 6.0f, -10.0f};
    const DirectX::XMFLOAT3 bottom{0.0f, -3.0f, -10.0f};

    auto addVertex = [&](const DirectX::XMFLOAT3& p, const DirectX::XMFLOAT3& n, float u, float v) {
        mesh.vertices.push_back(Vertex{p, n, DirectX::XMFLOAT2{u, v}, DirectX::XMFLOAT4{1.0f, 1.0f, 1.0f, 1.0f}});
    };

    // Main body tetra-like hull.
    addVertex(nose, {0, 1, 0}, 0.5f, 0.0f);
    addVertex(wingR, {0, 1, 0}, 1.0f, 0.5f);
    addVertex(top, {0, 1, 0}, 0.5f, 1.0f);

    addVertex(nose, {0, -1, 0}, 0.5f, 0.0f);
    addVertex(bottom, {0, -1, 0}, 0.5f, 1.0f);
    addVertex(wingR, {0, -1, 0}, 1.0f, 0.5f);

    addVertex(nose, {0, 1, 0}, 0.5f, 0.0f);
    addVertex(top, {0, 1, 0}, 0.5f, 1.0f);
    addVertex(wingL, {0, 1, 0}, 0.0f, 0.5f);

    addVertex(nose, {0, -1, 0}, 0.5f, 0.0f);
    addVertex(wingL, {0, -1, 0}, 0.0f, 0.5f);
    addVertex(bottom, {0, -1, 0}, 0.5f, 1.0f);

    addVertex(tail, {0, 1, 0}, 0.5f, 0.0f);
    addVertex(top, {0, 1, 0}, 1.0f, 1.0f);
    addVertex(bottom, {0, 1, 0}, 0.0f, 1.0f);

    mesh.indices.resize(mesh.vertices.size());
    for (uint32_t i = 0; i < mesh.indices.size(); ++i) {
        mesh.indices[i] = i;
    }

    return mesh;
}

} // namespace flight
