#include "gltf_loader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

#include <windows.h>
#include <wincodec.h>
#include <wrl/client.h>

#include "cgltf.h"

namespace flight {
namespace {

DirectX::XMFLOAT3 ToFloat3(const std::array<float, 3>& v) {
    return {v[0], v[1], -v[2]};
}

DirectX::XMFLOAT2 ToFloat2(const std::array<float, 2>& v) {
    return {v[0], v[1]};
}

std::array<float, 3> TransformPoint(const float* matrix, const std::array<float, 3>& v) {
    if (matrix == nullptr) {
        return v;
    }

    return {
        matrix[0] * v[0] + matrix[4] * v[1] + matrix[8] * v[2] + matrix[12],
        matrix[1] * v[0] + matrix[5] * v[1] + matrix[9] * v[2] + matrix[13],
        matrix[2] * v[0] + matrix[6] * v[1] + matrix[10] * v[2] + matrix[14],
    };
}

std::array<float, 3> TransformDirection(const float* matrix, const std::array<float, 3>& v) {
    if (matrix == nullptr) {
        return v;
    }

    return {
        matrix[0] * v[0] + matrix[4] * v[1] + matrix[8] * v[2],
        matrix[1] * v[0] + matrix[5] * v[1] + matrix[9] * v[2],
        matrix[2] * v[0] + matrix[6] * v[1] + matrix[10] * v[2],
    };
}

struct MaterialBakeInfo {
    DirectX::XMFLOAT4 colorFactor{1.0f, 1.0f, 1.0f, 1.0f};
    const GlbMaterialTexture* texture = nullptr;
};

DirectX::XMFLOAT4 SampleMaterialTexture(const GlbMaterialTexture& texture, const DirectX::XMFLOAT2& uv);

bool AppendPrimitive(
    const cgltf_primitive& primitive,
    const float* worldMatrix,
    const MaterialBakeInfo* bakeInfo,
    MeshData& outMesh,
    bool& anyMissingNormals,
    std::string& error) {
    if (primitive.type != cgltf_primitive_type_triangles) {
        return true;
    }

    const cgltf_accessor* posAccessor = nullptr;
    const cgltf_accessor* normalAccessor = nullptr;
    const cgltf_accessor* uvAccessor = nullptr;

    for (cgltf_size i = 0; i < primitive.attributes_count; ++i) {
        const cgltf_attribute& attr = primitive.attributes[i];
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
        error = "A triangle primitive is missing POSITION data";
        return false;
    }

    const size_t baseVertex = outMesh.vertices.size();
    const size_t vertexCount = static_cast<size_t>(posAccessor->count);
    outMesh.vertices.resize(baseVertex + vertexCount);

    for (size_t i = 0; i < vertexCount; ++i) {
        std::array<float, 3> pos{};
        cgltf_accessor_read_float(posAccessor, i, pos.data(), 3);
        pos = TransformPoint(worldMatrix, pos);
        outMesh.vertices[baseVertex + i].position = ToFloat3(pos);

        if (normalAccessor != nullptr) {
            std::array<float, 3> nrm{};
            cgltf_accessor_read_float(normalAccessor, i, nrm.data(), 3);
            nrm = TransformDirection(worldMatrix, nrm);
            DirectX::XMFLOAT3 normal = ToFloat3(nrm);
            const DirectX::XMVECTOR normalVec = DirectX::XMVector3Normalize(DirectX::XMLoadFloat3(&normal));
            DirectX::XMStoreFloat3(&outMesh.vertices[baseVertex + i].normal, normalVec);
        } else {
            outMesh.vertices[baseVertex + i].normal = {0.0f, 1.0f, 0.0f};
            anyMissingNormals = true;
        }

        if (uvAccessor != nullptr) {
            std::array<float, 2> uv{};
            cgltf_accessor_read_float(uvAccessor, i, uv.data(), 2);
            outMesh.vertices[baseVertex + i].uv = ToFloat2(uv);
        } else {
            outMesh.vertices[baseVertex + i].uv = {0.0f, 0.0f};
        }

        DirectX::XMFLOAT4 vertexColor{1.0f, 1.0f, 1.0f, 1.0f};
        if (bakeInfo != nullptr) {
            vertexColor = bakeInfo->colorFactor;
            if (bakeInfo->texture != nullptr) {
                const DirectX::XMFLOAT4 sampled = SampleMaterialTexture(*bakeInfo->texture, outMesh.vertices[baseVertex + i].uv);
                vertexColor.x *= sampled.x;
                vertexColor.y *= sampled.y;
                vertexColor.z *= sampled.z;
                vertexColor.w *= sampled.w;
            }
        }
        outMesh.vertices[baseVertex + i].color = vertexColor;
    }

    const size_t indexBase = outMesh.indices.size();
    if (primitive.indices != nullptr) {
        const size_t indexCount = static_cast<size_t>(primitive.indices->count);
        outMesh.indices.resize(indexBase + indexCount);
        for (size_t i = 0; i < indexCount; ++i) {
            outMesh.indices[indexBase + i] = static_cast<uint32_t>(baseVertex + cgltf_accessor_read_index(primitive.indices, i));
        }
    } else {
        outMesh.indices.resize(indexBase + vertexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            outMesh.indices[indexBase + i] = static_cast<uint32_t>(baseVertex + i);
        }
    }

    // GLTF is right-handed. Flip triangle winding for D3D12's left-handed setup.
    for (size_t i = indexBase; i + 2 < outMesh.indices.size(); i += 3) {
        std::swap(outMesh.indices[i + 1], outMesh.indices[i + 2]);
    }

    return true;
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

bool DecodeImageFromWicDecoder(
    IWICBitmapDecoder* decoder,
    std::vector<uint8_t>& outPixels,
    uint32_t& outWidth,
    uint32_t& outHeight) {
    Microsoft::WRL::ComPtr<IWICBitmapFrameDecode> frame;
    if (FAILED(decoder->GetFrame(0, frame.ReleaseAndGetAddressOf()))) {
        return false;
    }

    UINT w = 0;
    UINT h = 0;
    frame->GetSize(&w, &h);
    if (w == 0 || h == 0) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICImagingFactory> wicFactory;
    if (FAILED(CoCreateInstance(
            CLSID_WICImagingFactory,
            nullptr,
            CLSCTX_INPROC_SERVER,
            IID_PPV_ARGS(wicFactory.ReleaseAndGetAddressOf())))) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICFormatConverter> converter;
    if (FAILED(wicFactory->CreateFormatConverter(converter.ReleaseAndGetAddressOf()))) {
        return false;
    }

    if (FAILED(converter->Initialize(
            frame.Get(),
            GUID_WICPixelFormat32bppRGBA,
            WICBitmapDitherTypeNone,
            nullptr,
            0.0,
            WICBitmapPaletteTypeCustom))) {
        return false;
    }

    const UINT stride = w * 4;
    outPixels.resize(static_cast<size_t>(stride) * static_cast<size_t>(h));
    if (FAILED(converter->CopyPixels(nullptr, stride, static_cast<UINT>(outPixels.size()), outPixels.data()))) {
        return false;
    }

    outWidth = static_cast<uint32_t>(w);
    outHeight = static_cast<uint32_t>(h);
    return true;
}

bool DecodeImageFileToRgba(
    const std::filesystem::path& path,
    std::vector<uint8_t>& outPixels,
    uint32_t& outWidth,
    uint32_t& outHeight) {
    Microsoft::WRL::ComPtr<IWICImagingFactory> wicFactory;
    if (FAILED(CoCreateInstance(
            CLSID_WICImagingFactory,
            nullptr,
            CLSCTX_INPROC_SERVER,
            IID_PPV_ARGS(wicFactory.ReleaseAndGetAddressOf())))) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICBitmapDecoder> decoder;
    if (FAILED(wicFactory->CreateDecoderFromFilename(
            path.wstring().c_str(),
            nullptr,
            GENERIC_READ,
            WICDecodeMetadataCacheOnDemand,
            decoder.ReleaseAndGetAddressOf()))) {
        return false;
    }

    return DecodeImageFromWicDecoder(decoder.Get(), outPixels, outWidth, outHeight);
}

bool DecodeImageMemoryToRgba(
    const uint8_t* data,
    size_t sizeBytes,
    std::vector<uint8_t>& outPixels,
    uint32_t& outWidth,
    uint32_t& outHeight) {
    if (data == nullptr || sizeBytes == 0 || sizeBytes > static_cast<size_t>(std::numeric_limits<UINT>::max())) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICImagingFactory> wicFactory;
    if (FAILED(CoCreateInstance(
            CLSID_WICImagingFactory,
            nullptr,
            CLSCTX_INPROC_SERVER,
            IID_PPV_ARGS(wicFactory.ReleaseAndGetAddressOf())))) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICStream> stream;
    if (FAILED(wicFactory->CreateStream(stream.ReleaseAndGetAddressOf()))) {
        return false;
    }

    std::vector<uint8_t> temp(data, data + sizeBytes);
    if (FAILED(stream->InitializeFromMemory(temp.data(), static_cast<UINT>(temp.size())))) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICBitmapDecoder> decoder;
    if (FAILED(wicFactory->CreateDecoderFromStream(
            stream.Get(),
            nullptr,
            WICDecodeMetadataCacheOnLoad,
            decoder.ReleaseAndGetAddressOf()))) {
        return false;
    }

    return DecodeImageFromWicDecoder(decoder.Get(), outPixels, outWidth, outHeight);
}

DirectX::XMFLOAT4 SampleMaterialTexture(const GlbMaterialTexture& texture, const DirectX::XMFLOAT2& uv) {
    if (!texture.IsValid()) {
        return {1.0f, 1.0f, 1.0f, 1.0f};
    }

    const float uWrapped = uv.x - std::floor(uv.x);
    const float vWrapped = uv.y - std::floor(uv.y);
    const uint32_t x = std::min(
        texture.width - 1,
        static_cast<uint32_t>(std::floor(uWrapped * static_cast<float>(texture.width))));
    const uint32_t y = std::min(
        texture.height - 1,
        static_cast<uint32_t>(std::floor(vWrapped * static_cast<float>(texture.height))));
    const size_t texel = (static_cast<size_t>(y) * static_cast<size_t>(texture.width) + static_cast<size_t>(x)) * 4ull;
    return {
        texture.rgbaPixels[texel + 0] / 255.0f,
        texture.rgbaPixels[texel + 1] / 255.0f,
        texture.rgbaPixels[texel + 2] / 255.0f,
        texture.rgbaPixels[texel + 3] / 255.0f,
    };
}

MaterialBakeInfo ResolveMaterialBakeInfo(
    const cgltf_primitive& primitive,
    const std::filesystem::path& glbPath,
    std::unordered_map<const cgltf_image*, GlbMaterialTexture>& imageCache) {
    MaterialBakeInfo info{};
    if (primitive.material == nullptr) {
        return info;
    }

    const cgltf_material* material = primitive.material;
    const auto& pbr = material->pbr_metallic_roughness;
    info.colorFactor = {
        pbr.base_color_factor[0],
        pbr.base_color_factor[1],
        pbr.base_color_factor[2],
        pbr.base_color_factor[3],
    };

    const cgltf_texture* baseTex = pbr.base_color_texture.texture;
    if (baseTex == nullptr || baseTex->image == nullptr) {
        return info;
    }

    const cgltf_image* image = baseTex->image;
    auto it = imageCache.find(image);
    if (it == imageCache.end()) {
        GlbMaterialTexture decoded{};
        if (image->buffer_view != nullptr && image->buffer_view->buffer != nullptr && image->buffer_view->buffer->data != nullptr) {
            const cgltf_buffer_view* view = image->buffer_view;
            const uint8_t* raw = static_cast<const uint8_t*>(view->buffer->data) + view->offset;
            (void)DecodeImageMemoryToRgba(raw, static_cast<size_t>(view->size), decoded.rgbaPixels, decoded.width, decoded.height);
        } else if (image->uri != nullptr && image->uri[0] != '\0') {
            const std::filesystem::path uriPath = glbPath.parent_path() / image->uri;
            (void)DecodeImageFileToRgba(uriPath, decoded.rgbaPixels, decoded.width, decoded.height);
        }
        it = imageCache.emplace(image, std::move(decoded)).first;
    }

    if (it->second.IsValid()) {
        info.texture = &it->second;
    }
    return info;
}

bool LoadPrimitiveBaseColorTexture(
    const cgltf_primitive* primitive,
    const std::filesystem::path& glbPath,
    GlbMaterialTexture& outTexture) {
    outTexture = {};
    if (primitive == nullptr || primitive->material == nullptr) {
        return false;
    }

    const cgltf_material* mat = primitive->material;
    const cgltf_texture* baseTex = mat->pbr_metallic_roughness.base_color_texture.texture;
    if (baseTex == nullptr || baseTex->image == nullptr) {
        return false;
    }

    const cgltf_image* image = baseTex->image;
    std::vector<uint8_t> pixels;
    uint32_t width = 0;
    uint32_t height = 0;

    if (image->buffer_view != nullptr && image->buffer_view->buffer != nullptr && image->buffer_view->buffer->data != nullptr) {
        const cgltf_buffer_view* view = image->buffer_view;
        const uint8_t* raw = static_cast<const uint8_t*>(view->buffer->data) + view->offset;
        if (!DecodeImageMemoryToRgba(raw, static_cast<size_t>(view->size), pixels, width, height)) {
            return false;
        }
    } else if (image->uri != nullptr && image->uri[0] != '\0') {
        const std::filesystem::path uriPath = glbPath.parent_path() / image->uri;
        if (!DecodeImageFileToRgba(uriPath, pixels, width, height)) {
            return false;
        }
    } else {
        return false;
    }

    outTexture.rgbaPixels = std::move(pixels);
    outTexture.width = width;
    outTexture.height = height;
    return outTexture.IsValid();
}

} // namespace

bool LoadGlbMesh(
    const std::filesystem::path& glbPath,
    MeshData& outMesh,
    GlbMaterialTexture& outBaseColorTexture,
    bool bakeMaterialColor,
    std::string& error) {
    outMesh = {};
    outBaseColorTexture = {};

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

    bool appendedAny = false;
    bool anyMissingNormals = false;
    std::unordered_map<const cgltf_image*, GlbMaterialTexture> imageCache;

    for (cgltf_size nodeIndex = 0; nodeIndex < data->nodes_count; ++nodeIndex) {
        const cgltf_node& node = data->nodes[nodeIndex];
        if (node.mesh == nullptr) {
            continue;
        }

        float worldMatrix[16]{};
        cgltf_node_transform_world(&node, worldMatrix);
        for (cgltf_size primitiveIndex = 0; primitiveIndex < node.mesh->primitives_count; ++primitiveIndex) {
            const cgltf_primitive& primitive = node.mesh->primitives[primitiveIndex];
            MaterialBakeInfo bakeInfo{};
            const MaterialBakeInfo* bakeInfoPtr = nullptr;
            if (bakeMaterialColor) {
                bakeInfo = ResolveMaterialBakeInfo(primitive, glbPath, imageCache);
                bakeInfoPtr = &bakeInfo;
            } else if (!outBaseColorTexture.IsValid()) {
                (void)LoadPrimitiveBaseColorTexture(&primitive, glbPath, outBaseColorTexture);
            }
            if (!AppendPrimitive(primitive, worldMatrix, bakeInfoPtr, outMesh, anyMissingNormals, error)) {
                cgltf_free(data);
                return false;
            }
            if (primitive.type == cgltf_primitive_type_triangles) {
                appendedAny = true;
            }
        }
    }

    if (!appendedAny) {
        for (cgltf_size meshIndex = 0; meshIndex < data->meshes_count; ++meshIndex) {
            const cgltf_mesh& mesh = data->meshes[meshIndex];
            for (cgltf_size primitiveIndex = 0; primitiveIndex < mesh.primitives_count; ++primitiveIndex) {
                const cgltf_primitive& primitive = mesh.primitives[primitiveIndex];
                MaterialBakeInfo bakeInfo{};
                const MaterialBakeInfo* bakeInfoPtr = nullptr;
                if (bakeMaterialColor) {
                    bakeInfo = ResolveMaterialBakeInfo(primitive, glbPath, imageCache);
                    bakeInfoPtr = &bakeInfo;
                } else if (!outBaseColorTexture.IsValid()) {
                    (void)LoadPrimitiveBaseColorTexture(&primitive, glbPath, outBaseColorTexture);
                }
                if (!AppendPrimitive(primitive, nullptr, bakeInfoPtr, outMesh, anyMissingNormals, error)) {
                    cgltf_free(data);
                    return false;
                }
                if (primitive.type == cgltf_primitive_type_triangles) {
                    appendedAny = true;
                }
            }
        }
    }

    if (!appendedAny) {
        cgltf_free(data);
        error = "No triangle primitives found in GLB";
        return false;
    }

    if (anyMissingNormals) {
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

bool LoadGlbMesh(const std::filesystem::path& glbPath, MeshData& outMesh, std::string& error) {
    GlbMaterialTexture ignoredTexture;
    return LoadGlbMesh(glbPath, outMesh, ignoredTexture, false, error);
}

bool LoadGlbMesh(
    const std::filesystem::path& glbPath,
    MeshData& outMesh,
    GlbMaterialTexture& outBaseColorTexture,
    std::string& error) {
    return LoadGlbMesh(glbPath, outMesh, outBaseColorTexture, false, error);
}

} // namespace flight
