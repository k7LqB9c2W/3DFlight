#include "gltf_loader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "cgltf.h"

namespace flight {
namespace {

DirectX::XMFLOAT3 ToFloat3(const std::array<float, 3>& v) {
    return {v[0], v[1], -v[2]};
}

DirectX::XMFLOAT2 ToFloat2(const std::array<float, 2>& v) {
    return {v[0], v[1]};
}

void ComputeNormals(MeshData& mesh) {
    std::vector<DirectX::XMFLOAT3> sums(mesh.vertices.size(), {0.0f, 0.0f, 0.0f});

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const uint32_t i0 = mesh.indices[i + 0];
        const uint32_t i1 = mesh.indices[i + 1];
        const uint32_t i2 = mesh.indices[i + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) {
            continue;
        }

        const DirectX::XMVECTOR p0 = DirectX::XMLoadFloat3(&mesh.vertices[i0].position);
        const DirectX::XMVECTOR p1 = DirectX::XMLoadFloat3(&mesh.vertices[i1].position);
        const DirectX::XMVECTOR p2 = DirectX::XMLoadFloat3(&mesh.vertices[i2].position);

        const DirectX::XMVECTOR n = DirectX::XMVector3Cross(
            DirectX::XMVectorSubtract(p1, p0),
            DirectX::XMVectorSubtract(p2, p0));

        DirectX::XMVECTOR s0 = DirectX::XMLoadFloat3(&sums[i0]);
        DirectX::XMVECTOR s1 = DirectX::XMLoadFloat3(&sums[i1]);
        DirectX::XMVECTOR s2 = DirectX::XMLoadFloat3(&sums[i2]);

        s0 = DirectX::XMVectorAdd(s0, n);
        s1 = DirectX::XMVectorAdd(s1, n);
        s2 = DirectX::XMVectorAdd(s2, n);

        DirectX::XMStoreFloat3(&sums[i0], s0);
        DirectX::XMStoreFloat3(&sums[i1], s1);
        DirectX::XMStoreFloat3(&sums[i2], s2);
    }

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        DirectX::XMVECTOR n = DirectX::XMLoadFloat3(&sums[i]);
        n = DirectX::XMVector3Normalize(n);
        DirectX::XMStoreFloat3(&mesh.vertices[i].normal, n);
    }
}

void NormalizeScaleAndCenter(MeshData& mesh, float targetLengthMeters) {
    if (mesh.vertices.empty()) {
        return;
    }

    DirectX::XMFLOAT3 minP = mesh.vertices[0].position;
    DirectX::XMFLOAT3 maxP = mesh.vertices[0].position;

    for (const auto& v : mesh.vertices) {
        minP.x = std::min(minP.x, v.position.x);
        minP.y = std::min(minP.y, v.position.y);
        minP.z = std::min(minP.z, v.position.z);
        maxP.x = std::max(maxP.x, v.position.x);
        maxP.y = std::max(maxP.y, v.position.y);
        maxP.z = std::max(maxP.z, v.position.z);
    }

    const float extentX = maxP.x - minP.x;
    const float extentY = maxP.y - minP.y;
    const float extentZ = maxP.z - minP.z;
    const float extent = std::max({extentX, extentY, extentZ, 1e-3f});

    const float scale = targetLengthMeters / extent;
    const DirectX::XMFLOAT3 center{
        0.5f * (minP.x + maxP.x),
        0.5f * (minP.y + maxP.y),
        0.5f * (minP.z + maxP.z),
    };

    for (auto& v : mesh.vertices) {
        v.position.x = (v.position.x - center.x) * scale;
        v.position.y = (v.position.y - center.y) * scale;
        v.position.z = (v.position.z - center.z) * scale;
    }
}

} // namespace

bool LoadGlbMesh(const std::filesystem::path& glbPath, MeshData& outMesh, std::string& error) {
    outMesh = {};

    if (!std::filesystem::exists(glbPath)) {
        error = "GLB file not found: " + glbPath.string();
        return false;
    }

    cgltf_options options{};
    cgltf_data* data = nullptr;
    const std::string fileUtf8 = glbPath.string();
    cgltf_result result = cgltf_parse_file(&options, fileUtf8.c_str(), &data);
    if (result != cgltf_result_success) {
        error = "cgltf_parse_file failed";
        return false;
    }

    const std::string parentUtf8 = glbPath.parent_path().string();
    result = cgltf_load_buffers(&options, data, parentUtf8.c_str());
    if (result != cgltf_result_success) {
        cgltf_free(data);
        error = "cgltf_load_buffers failed";
        return false;
    }

    const cgltf_primitive* primitive = nullptr;
    for (cgltf_size m = 0; m < data->meshes_count && primitive == nullptr; ++m) {
        const cgltf_mesh& mesh = data->meshes[m];
        if (mesh.primitives_count > 0) {
            primitive = &mesh.primitives[0];
        }
    }

    if (primitive == nullptr) {
        cgltf_free(data);
        error = "No primitive found in GLB";
        return false;
    }

    if (primitive->type != cgltf_primitive_type_triangles) {
        cgltf_free(data);
        error = "Only triangle primitives are supported";
        return false;
    }

    const cgltf_accessor* posAccessor = nullptr;
    const cgltf_accessor* normalAccessor = nullptr;
    const cgltf_accessor* uvAccessor = nullptr;

    for (cgltf_size i = 0; i < primitive->attributes_count; ++i) {
        const cgltf_attribute& attr = primitive->attributes[i];
        switch (attr.type) {
            case cgltf_attribute_type_position:
                posAccessor = attr.data;
                break;
            case cgltf_attribute_type_normal:
                normalAccessor = attr.data;
                break;
            case cgltf_attribute_type_texcoord:
                if (attr.index == 0) {
                    uvAccessor = attr.data;
                }
                break;
            default:
                break;
        }
    }

    if (posAccessor == nullptr) {
        cgltf_free(data);
        error = "No POSITION accessor found";
        return false;
    }

    const size_t vertexCount = static_cast<size_t>(posAccessor->count);
    outMesh.vertices.resize(vertexCount);

    for (size_t i = 0; i < vertexCount; ++i) {
        std::array<float, 3> pos{};
        cgltf_accessor_read_float(posAccessor, i, pos.data(), 3);
        outMesh.vertices[i].position = ToFloat3(pos);

        if (normalAccessor != nullptr) {
            std::array<float, 3> nrm{};
            cgltf_accessor_read_float(normalAccessor, i, nrm.data(), 3);
            outMesh.vertices[i].normal = ToFloat3(nrm);
        } else {
            outMesh.vertices[i].normal = {0.0f, 1.0f, 0.0f};
        }

        if (uvAccessor != nullptr) {
            std::array<float, 2> uv{};
            cgltf_accessor_read_float(uvAccessor, i, uv.data(), 2);
            outMesh.vertices[i].uv = ToFloat2(uv);
        } else {
            outMesh.vertices[i].uv = {0.0f, 0.0f};
        }
    }

    if (primitive->indices != nullptr) {
        const size_t indexCount = static_cast<size_t>(primitive->indices->count);
        outMesh.indices.resize(indexCount);
        for (size_t i = 0; i < indexCount; ++i) {
            outMesh.indices[i] = static_cast<uint32_t>(cgltf_accessor_read_index(primitive->indices, i));
        }
    } else {
        outMesh.indices.resize(vertexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            outMesh.indices[i] = static_cast<uint32_t>(i);
        }
    }

    // GLTF is right-handed. Flip triangle winding for D3D12's left-handed setup.
    for (size_t i = 0; i + 2 < outMesh.indices.size(); i += 3) {
        std::swap(outMesh.indices[i + 1], outMesh.indices[i + 2]);
    }

    if (normalAccessor == nullptr) {
        ComputeNormals(outMesh);
    }

    NormalizeScaleAndCenter(outMesh, 60.0f);

    cgltf_free(data);

    if (!outMesh.IsValid()) {
        error = "Loaded GLB mesh is empty";
        return false;
    }

    return true;
}

} // namespace flight
