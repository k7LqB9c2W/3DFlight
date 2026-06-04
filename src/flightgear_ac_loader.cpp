#include "flightgear_ac_loader.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <windows.h>
#include <wincodec.h>
#include <wrl/client.h>

namespace flight {
namespace {

struct Float3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Float2 {
    float x = 0.0f;
    float y = 0.0f;
};

struct Matrix4 {
    float m[4][4]{};
};

struct AcMaterial {
    DirectX::XMFLOAT4 color{1.0f, 1.0f, 1.0f, 1.0f};
};

struct TextureImage {
    std::vector<uint8_t> pixels;
    uint32_t width = 0;
    uint32_t height = 0;

    [[nodiscard]] bool IsValid() const {
        return width > 0 && height > 0 && pixels.size() == static_cast<size_t>(width) * static_cast<size_t>(height) * 4ull;
    }
};

struct ParseContext {
    std::vector<AcMaterial> materials;
    std::unordered_map<std::wstring, TextureImage> textureCache;
    std::filesystem::path aircraftRoot;
    std::unordered_set<std::wstring> loadedXml;
    FlightGearLoadStats stats{};
};

struct SubmodelOffset {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float headingDeg = 0.0f;
};

struct XmlModelRef {
    std::string path;
    SubmodelOffset offset;
};

Matrix4 Identity() {
    Matrix4 out{};
    out.m[0][0] = 1.0f;
    out.m[1][1] = 1.0f;
    out.m[2][2] = 1.0f;
    out.m[3][3] = 1.0f;
    return out;
}

Matrix4 Multiply(const Matrix4& a, const Matrix4& b) {
    Matrix4 out{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            for (int k = 0; k < 4; ++k) {
                out.m[r][c] += a.m[r][k] * b.m[k][c];
            }
        }
    }
    return out;
}

Float3 TransformPoint(const Matrix4& matrix, const Float3& p) {
    return {
        matrix.m[0][0] * p.x + matrix.m[0][1] * p.y + matrix.m[0][2] * p.z + matrix.m[0][3],
        matrix.m[1][0] * p.x + matrix.m[1][1] * p.y + matrix.m[1][2] * p.z + matrix.m[1][3],
        matrix.m[2][0] * p.x + matrix.m[2][1] * p.y + matrix.m[2][2] * p.z + matrix.m[2][3],
    };
}

Float3 Add(const Float3& a, const Float3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Float3 Sub(const Float3& a, const Float3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Float3 Cross(const Float3& a, const Float3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

Float3 Normalize(const Float3& v) {
    const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len <= 1e-8f) {
        return {0.0f, 1.0f, 0.0f};
    }
    return {v.x / len, v.y / len, v.z / len};
}

Matrix4 Translation(float x, float y, float z) {
    Matrix4 out = Identity();
    out.m[0][3] = x;
    out.m[1][3] = y;
    out.m[2][3] = z;
    return out;
}

Matrix4 RotationAxis(Float3 axis, float radians) {
    axis = Normalize(axis);
    const float c = std::cos(radians);
    const float s = std::sin(radians);
    const float t = 1.0f - c;
    Matrix4 out = Identity();
    out.m[0][0] = t * axis.x * axis.x + c;
    out.m[0][1] = t * axis.x * axis.y - s * axis.z;
    out.m[0][2] = t * axis.x * axis.z + s * axis.y;
    out.m[1][0] = t * axis.x * axis.y + s * axis.z;
    out.m[1][1] = t * axis.y * axis.y + c;
    out.m[1][2] = t * axis.y * axis.z - s * axis.x;
    out.m[2][0] = t * axis.x * axis.z - s * axis.y;
    out.m[2][1] = t * axis.y * axis.z + s * axis.x;
    out.m[2][2] = t * axis.z * axis.z + c;
    return out;
}

Matrix4 OffsetMatrix(const SubmodelOffset& offset) {
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
    Matrix4 out = Translation(offset.x, offset.y, offset.z);
    if (std::abs(offset.headingDeg) > 1e-6f) {
        out = Multiply(out, RotationAxis({0.0f, 0.0f, 1.0f}, offset.headingDeg * kDegToRad));
    }
    if (std::abs(offset.pitchDeg) > 1e-6f) {
        out = Multiply(out, RotationAxis({0.0f, 1.0f, 0.0f}, offset.pitchDeg * kDegToRad));
    }
    if (std::abs(offset.rollDeg) > 1e-6f) {
        out = Multiply(out, RotationAxis({1.0f, 0.0f, 0.0f}, offset.rollDeg * kDegToRad));
    }
    return out;
}

std::string Trim(std::string s) {
    const auto first = std::find_if_not(s.begin(), s.end(), [](unsigned char ch) { return std::isspace(ch) != 0; });
    const auto last = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::string ReadLineString(std::istream& stream) {
    std::string line;
    std::getline(stream, line);
    line = Trim(line);
    if (line.size() >= 2 && line.front() == '"' && line.back() == '"') {
        return line.substr(1, line.size() - 2);
    }
    return line;
}

std::string Lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return s;
}

bool StartsWith(const std::string& value, const std::string& prefix) {
    return value.rfind(prefix, 0) == 0;
}

bool ContainsInsensitive(const std::string& value, const std::string& needle) {
    return Lower(value).find(Lower(needle)) != std::string::npos;
}

std::string GetFirstTagText(const std::string& xml, const std::string& tag) {
    const std::regex pattern("<\\s*" + tag + "(?:\\s+[^>]*)?\\s*>([\\s\\S]*?)<\\s*/\\s*" + tag + "\\s*>", std::regex::icase);
    std::smatch match;
    if (std::regex_search(xml, match, pattern) && match.size() >= 2) {
        return Trim(match[1].str());
    }
    return {};
}

float GetFirstTagFloat(const std::string& xml, const std::string& tag, float fallback = 0.0f) {
    const std::string value = GetFirstTagText(xml, tag);
    if (value.empty()) {
        return fallback;
    }
    try {
        return std::stof(value);
    } catch (...) {
        return fallback;
    }
}

std::vector<std::string> ExtractBlocks(const std::string& xml, const std::string& tag) {
    std::vector<std::string> out;
    const std::regex pattern("<\\s*" + tag + "(?:\\s+[^>]*)?\\s*>([\\s\\S]*?)<\\s*/\\s*" + tag + "\\s*>", std::regex::icase);
    for (std::sregex_iterator it(xml.begin(), xml.end(), pattern), end; it != end; ++it) {
        out.push_back((*it)[1].str());
    }
    return out;
}

std::filesystem::path ResolveFlightGearPath(
    const std::filesystem::path& currentXml,
    const std::filesystem::path& aircraftRoot,
    const std::string& rawPath) {
    std::string normalized = rawPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    constexpr const char* kAircraftPrefix = "Aircraft/737-800YV/";
    if (StartsWith(normalized, kAircraftPrefix)) {
        normalized = normalized.substr(std::char_traits<char>::length(kAircraftPrefix));
        return aircraftRoot / std::filesystem::path(normalized);
    }
    if (StartsWith(normalized, "Models/") || StartsWith(normalized, "Sounds/") || StartsWith(normalized, "Effects/")) {
        return aircraftRoot / std::filesystem::path(normalized);
    }
    return currentXml.parent_path() / std::filesystem::path(normalized);
}

std::optional<std::filesystem::path> FindTextureFile(const std::filesystem::path& acPath, const std::string& textureName) {
    std::vector<std::filesystem::path> candidates;
    candidates.push_back(acPath.parent_path() / textureName);
    candidates.push_back(acPath.parent_path().parent_path() / textureName);
    for (const auto& candidate : candidates) {
        std::error_code ec;
        if (std::filesystem::exists(candidate, ec)) {
            return candidate;
        }
    }
    return std::nullopt;
}

bool DecodeImageFileWic(const std::filesystem::path& path, TextureImage& out) {
    Microsoft::WRL::ComPtr<IWICImagingFactory> factory;
    if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(factory.ReleaseAndGetAddressOf())))) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICBitmapDecoder> decoder;
    if (FAILED(factory->CreateDecoderFromFilename(path.wstring().c_str(), nullptr, GENERIC_READ, WICDecodeMetadataCacheOnDemand, decoder.ReleaseAndGetAddressOf()))) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICBitmapFrameDecode> frame;
    if (FAILED(decoder->GetFrame(0, frame.ReleaseAndGetAddressOf()))) {
        return false;
    }

    UINT width = 0;
    UINT height = 0;
    if (FAILED(frame->GetSize(&width, &height)) || width == 0 || height == 0) {
        return false;
    }

    Microsoft::WRL::ComPtr<IWICFormatConverter> converter;
    if (FAILED(factory->CreateFormatConverter(converter.ReleaseAndGetAddressOf()))) {
        return false;
    }
    if (FAILED(converter->Initialize(frame.Get(), GUID_WICPixelFormat32bppRGBA, WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom))) {
        return false;
    }

    std::vector<uint8_t> pixels(static_cast<size_t>(width) * static_cast<size_t>(height) * 4ull);
    const UINT stride = width * 4u;
    if (FAILED(converter->CopyPixels(nullptr, stride, static_cast<UINT>(pixels.size()), pixels.data()))) {
        return false;
    }

    out.pixels = std::move(pixels);
    out.width = width;
    out.height = height;
    return out.IsValid();
}

uint16_t ReadBe16(std::istream& in) {
    unsigned char b[2]{};
    in.read(reinterpret_cast<char*>(b), 2);
    return static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8u) | b[1]);
}

uint32_t ReadBe32(std::istream& in) {
    unsigned char b[4]{};
    in.read(reinterpret_cast<char*>(b), 4);
    return (static_cast<uint32_t>(b[0]) << 24u) | (static_cast<uint32_t>(b[1]) << 16u) | (static_cast<uint32_t>(b[2]) << 8u) |
        static_cast<uint32_t>(b[3]);
}

bool DecodeSgiRgb(const std::filesystem::path& path, TextureImage& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }

    const uint16_t magic = ReadBe16(in);
    const uint8_t storage = static_cast<uint8_t>(in.get());
    const uint8_t bpc = static_cast<uint8_t>(in.get());
    const uint16_t dimension = ReadBe16(in);
    const uint16_t width = ReadBe16(in);
    const uint16_t height = ReadBe16(in);
    const uint16_t channels = ReadBe16(in);
    if (magic != 474 || bpc != 1 || width == 0 || height == 0 || channels == 0 || channels > 4 || dimension == 0) {
        return false;
    }

    in.seekg(512, std::ios::beg);
    std::vector<std::vector<uint8_t>> planes(channels, std::vector<uint8_t>(static_cast<size_t>(width) * height));

    if (storage == 0) {
        for (uint16_t z = 0; z < channels; ++z) {
            for (uint16_t y = 0; y < height; ++y) {
                in.read(reinterpret_cast<char*>(planes[z].data() + static_cast<size_t>(y) * width), width);
            }
        }
    } else if (storage == 1) {
        const size_t rows = static_cast<size_t>(height) * channels;
        std::vector<uint32_t> rowStart(rows);
        std::vector<uint32_t> rowSize(rows);
        in.seekg(512, std::ios::beg);
        for (size_t i = 0; i < rows; ++i) {
            rowStart[i] = ReadBe32(in);
        }
        for (size_t i = 0; i < rows; ++i) {
            rowSize[i] = ReadBe32(in);
        }
        for (uint16_t z = 0; z < channels; ++z) {
            for (uint16_t y = 0; y < height; ++y) {
                const size_t row = static_cast<size_t>(z) * height + y;
                in.seekg(rowStart[row], std::ios::beg);
                uint8_t* dst = planes[z].data() + static_cast<size_t>(y) * width;
                size_t x = 0;
                while (x < width && in) {
                    const uint8_t packet = static_cast<uint8_t>(in.get());
                    const uint8_t count = packet & 0x7f;
                    if (count == 0) {
                        break;
                    }
                    if ((packet & 0x80) != 0) {
                        for (uint8_t i = 0; i < count && x < width; ++i) {
                            dst[x++] = static_cast<uint8_t>(in.get());
                        }
                    } else {
                        const uint8_t value = static_cast<uint8_t>(in.get());
                        for (uint8_t i = 0; i < count && x < width; ++i) {
                            dst[x++] = value;
                        }
                    }
                }
            }
        }
    } else {
        return false;
    }

    std::vector<uint8_t> rgba(static_cast<size_t>(width) * height * 4ull);
    for (uint16_t y = 0; y < height; ++y) {
        for (uint16_t x = 0; x < width; ++x) {
            const size_t src = static_cast<size_t>(y) * width + x;
            const size_t dst = src * 4ull;
            rgba[dst + 0] = planes[0][src];
            rgba[dst + 1] = channels >= 2 ? planes[1][src] : planes[0][src];
            rgba[dst + 2] = channels >= 3 ? planes[2][src] : planes[0][src];
            rgba[dst + 3] = channels >= 4 ? planes[3][src] : 255;
        }
    }

    out.pixels = std::move(rgba);
    out.width = width;
    out.height = height;
    return out.IsValid();
}

const TextureImage* LoadTexture(ParseContext& context, const std::filesystem::path& acPath, const std::string& textureName) {
    const auto texturePath = FindTextureFile(acPath, textureName);
    if (!texturePath) {
        context.stats.texturesMissing += 1;
        return nullptr;
    }

    const std::wstring key = std::filesystem::weakly_canonical(*texturePath).wstring();
    const auto found = context.textureCache.find(key);
    if (found != context.textureCache.end()) {
        return found->second.IsValid() ? &found->second : nullptr;
    }

    TextureImage image;
    const std::string ext = Lower(texturePath->extension().string());
    const bool ok = (ext == ".rgb") ? DecodeSgiRgb(*texturePath, image) : DecodeImageFileWic(*texturePath, image);
    if (ok) {
        context.stats.texturesLoaded += 1;
    } else {
        context.stats.texturesMissing += 1;
    }
    auto [it, _] = context.textureCache.emplace(key, std::move(image));
    return it->second.IsValid() ? &it->second : nullptr;
}

DirectX::XMFLOAT4 SampleTexture(const TextureImage* texture, const Float2& uv) {
    if (texture == nullptr || !texture->IsValid()) {
        return {1.0f, 1.0f, 1.0f, 1.0f};
    }
    float u = uv.x - std::floor(uv.x);
    float v = uv.y - std::floor(uv.y);
    const uint32_t x = std::min(texture->width - 1u, static_cast<uint32_t>(u * static_cast<float>(texture->width)));
    const uint32_t y = std::min(texture->height - 1u, static_cast<uint32_t>((1.0f - v) * static_cast<float>(texture->height)));
    const size_t idx = (static_cast<size_t>(y) * texture->width + x) * 4ull;
    return {
        static_cast<float>(texture->pixels[idx + 0]) / 255.0f,
        static_cast<float>(texture->pixels[idx + 1]) / 255.0f,
        static_cast<float>(texture->pixels[idx + 2]) / 255.0f,
        static_cast<float>(texture->pixels[idx + 3]) / 255.0f,
    };
}

void AppendTriangle(MeshData& mesh, const std::array<Float3, 3>& positions, const std::array<Float2, 3>& uvs, const DirectX::XMFLOAT4& color) {
    const Float3 normal = Normalize(Cross(Sub(positions[1], positions[0]), Sub(positions[2], positions[0])));
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (size_t i = 0; i < 3; ++i) {
        // FlightGear/AC3D aircraft use negative X forward, Y up, Z lateral. The renderer mesh convention is X right, Y up, Z forward.
        Vertex v{};
        v.position = {positions[i].z, positions[i].y, -positions[i].x};
        v.normal = {normal.z, normal.y, -normal.x};
        v.uv = {uvs[i].x, uvs[i].y};
        v.color = color;
        mesh.vertices.push_back(v);
    }
    mesh.indices.push_back(base + 0u);
    mesh.indices.push_back(base + 1u);
    mesh.indices.push_back(base + 2u);
}

AcMaterial ParseMaterial(const std::string& line) {
    AcMaterial material{};
    std::istringstream stream(line);
    std::string token;
    float trans = 0.0f;
    while (stream >> token) {
        if (token == "rgb") {
            stream >> material.color.x >> material.color.y >> material.color.z;
        } else if (token == "trans") {
            stream >> trans;
        }
    }
    material.color.w = std::clamp(1.0f - trans, 0.0f, 1.0f);
    return material;
}

bool ParseAcObject(
    std::istream& stream,
    ParseContext& context,
    const std::filesystem::path& acPath,
    const Matrix4& parentTransform,
    const TextureImage* inheritedTexture,
    MeshData& mesh,
    std::string& error) {
    std::string objectType;
    stream >> objectType;

    Matrix4 local = Identity();
    Matrix4 currentTransform = parentTransform;
    std::vector<Float3> vertices;
    const TextureImage* objectTexture = inheritedTexture;
    Float2 textureRepeat{1.0f, 1.0f};
    Float2 textureOffset{0.0f, 0.0f};

    while (stream.good()) {
        std::string token;
        stream >> token;
        if (token.empty()) {
            continue;
        }

        if (token == "OBJECT") {
            error = "Unexpected OBJECT before kids in " + acPath.string();
            return false;
        }
        if (token == "MATERIAL") {
            const std::string rest = ReadLineString(stream);
            context.materials.push_back(ParseMaterial(rest));
        } else if (token == "name") {
            (void)ReadLineString(stream);
        } else if (token == "data") {
            int len = 0;
            stream >> len;
            if (len > 0) {
                std::vector<char> ignored(static_cast<size_t>(len));
                stream.read(ignored.data(), len);
            }
        } else if (token == "texture") {
            objectTexture = LoadTexture(context, acPath, ReadLineString(stream));
        } else if (token == "texrep") {
            stream >> textureRepeat.x >> textureRepeat.y;
        } else if (token == "texoff") {
            stream >> textureOffset.x >> textureOffset.y;
        } else if (token == "rot") {
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    stream >> local.m[r][c];
                }
            }
            currentTransform = Multiply(parentTransform, local);
        } else if (token == "loc") {
            stream >> local.m[0][3] >> local.m[1][3] >> local.m[2][3];
            currentTransform = Multiply(parentTransform, local);
        } else if (token == "crease" || token == "url") {
            std::string ignored;
            std::getline(stream, ignored);
        } else if (token == "numvert") {
            uint32_t count = 0;
            stream >> count;
            vertices.clear();
            vertices.reserve(count);
            for (uint32_t i = 0; i < count; ++i) {
                Float3 p{};
                stream >> p.x >> p.y >> p.z;
                vertices.push_back(TransformPoint(currentTransform, p));
            }
        } else if (token == "numsurf") {
            uint32_t surfaceCount = 0;
            stream >> surfaceCount;
            for (uint32_t surface = 0; surface < surfaceCount; ++surface) {
                std::string surfToken;
                stream >> surfToken;
                if (surfToken != "SURF") {
                    error = "Expected SURF in " + acPath.string();
                    return false;
                }
                std::string flagsText;
                stream >> flagsText;
                std::string matToken;
                uint32_t matIndex = 0;
                stream >> matToken >> matIndex;
                if (matToken != "mat") {
                    error = "Expected mat in " + acPath.string();
                    return false;
                }
                std::string refsToken;
                uint32_t refCount = 0;
                stream >> refsToken >> refCount;
                if (refsToken != "refs") {
                    error = "Expected refs in " + acPath.string();
                    return false;
                }

                struct Ref {
                    uint32_t index = 0;
                    Float2 uv{};
                };
                std::vector<Ref> refs(refCount);
                for (uint32_t i = 0; i < refCount; ++i) {
                    stream >> refs[i].index >> refs[i].uv.x >> refs[i].uv.y;
                    refs[i].uv.x = textureOffset.x + refs[i].uv.x * textureRepeat.x;
                    refs[i].uv.y = textureOffset.y + refs[i].uv.y * textureRepeat.y;
                }
                if (refCount < 3) {
                    continue;
                }
                DirectX::XMFLOAT4 baseColor{1.0f, 1.0f, 1.0f, 1.0f};
                if (matIndex < context.materials.size()) {
                    baseColor = context.materials[matIndex].color;
                }
                const DirectX::XMFLOAT4 sampled = SampleTexture(objectTexture, refs[0].uv);
                baseColor.x *= sampled.x;
                baseColor.y *= sampled.y;
                baseColor.z *= sampled.z;
                baseColor.w *= sampled.w;

                for (uint32_t i = 1; i + 1 < refCount; ++i) {
                    if (refs[0].index >= vertices.size() || refs[i].index >= vertices.size() || refs[i + 1].index >= vertices.size()) {
                        continue;
                    }
                    AppendTriangle(
                        mesh,
                        {vertices[refs[0].index], vertices[refs[i].index], vertices[refs[i + 1].index]},
                        {refs[0].uv, refs[i].uv, refs[i + 1].uv},
                        baseColor);
                }
            }
        } else if (token == "kids") {
            uint32_t childCount = 0;
            stream >> childCount;
            for (uint32_t i = 0; i < childCount; ++i) {
                std::string childToken;
                stream >> childToken;
                if (childToken != "OBJECT") {
                    error = "Expected child OBJECT in " + acPath.string();
                    return false;
                }
                if (!ParseAcObject(stream, context, acPath, currentTransform, objectTexture, mesh, error)) {
                    return false;
                }
            }
            return true;
        } else {
            std::string ignored;
            std::getline(stream, ignored);
        }
    }
    return true;
}

bool LoadAcFile(
    const std::filesystem::path& acPath,
    const Matrix4& transform,
    ParseContext& context,
    MeshData& mesh,
    std::string& error) {
    std::ifstream stream(acPath);
    if (!stream) {
        error = "AC3D file not found: " + acPath.string();
        return false;
    }
    std::string header;
    stream >> header;
    if (header.rfind("AC3D", 0) != 0) {
        error = "Not an AC3D file: " + acPath.string();
        return false;
    }

    context.materials.clear();
    while (stream.good()) {
        std::string token;
        stream >> token;
        if (token.empty()) {
            continue;
        }
        if (token == "MATERIAL") {
            context.materials.push_back(ParseMaterial(ReadLineString(stream)));
        } else if (token == "OBJECT") {
            if (!ParseAcObject(stream, context, acPath, transform, nullptr, mesh, error)) {
                return false;
            }
        } else {
            std::string ignored;
            std::getline(stream, ignored);
        }
    }
    context.stats.acFilesLoaded += 1;
    return true;
}

bool ShouldIncludeModelPath(const std::string& path, const FlightGearLoadOptions& options) {
    std::string normalized = path;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::string lower = Lower(normalized);
    if (lower.find("/services/") != std::string::npos || lower.find("/autopush/") != std::string::npos || lower.find("/effects/") != std::string::npos) {
        return false;
    }
    if (lower.find("/lights/") != std::string::npos || lower.find("light-cone") != std::string::npos) {
        return false;
    }
    if (lower.find("pushback") != std::string::npos || lower.find("tyre-smoke") != std::string::npos || lower.find("tyrespray") != std::string::npos ||
        lower.find("contrail") != std::string::npos || lower.find("fire") != std::string::npos) {
        return false;
    }
    if (lower.find("/cockpit") != std::string::npos || lower.find("cockpit.xml") != std::string::npos) {
        return options.includeCockpit;
    }
    if (lower.find("/seats/") != std::string::npos || lower.find("passenger-seating") != std::string::npos) {
        return options.includePassengerSeats;
    }
    return true;
}

std::vector<XmlModelRef> ParseModelRefs(const std::string& xml) {
    std::vector<XmlModelRef> out;
    for (const std::string& block : ExtractBlocks(xml, "model")) {
        XmlModelRef ref{};
        ref.path = GetFirstTagText(block, "path");
        const std::string offsets = GetFirstTagText(block, "offsets");
        if (!offsets.empty()) {
            ref.offset.x = GetFirstTagFloat(offsets, "x-m");
            ref.offset.y = GetFirstTagFloat(offsets, "y-m");
            ref.offset.z = GetFirstTagFloat(offsets, "z-m");
            ref.offset.rollDeg = GetFirstTagFloat(offsets, "roll-deg");
            ref.offset.pitchDeg = GetFirstTagFloat(offsets, "pitch-deg");
            ref.offset.headingDeg = GetFirstTagFloat(offsets, "heading-deg");
        }
        if (!ref.path.empty()) {
            out.push_back(ref);
        }
    }
    return out;
}

bool LoadModelXmlRecursive(
    const std::filesystem::path& xmlPath,
    const Matrix4& transform,
    ParseContext& context,
    const FlightGearLoadOptions& options,
    MeshData& mesh,
    std::string& error) {
    std::error_code ec;
    const std::filesystem::path canonical = std::filesystem::weakly_canonical(xmlPath, ec);
    const std::wstring key = ec ? xmlPath.wstring() : canonical.wstring();
    if (!context.loadedXml.insert(key).second) {
        return true;
    }

    std::ifstream file(xmlPath);
    if (!file) {
        error = "Model XML not found: " + xmlPath.string();
        return false;
    }
    const std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    context.stats.xmlFilesLoaded += 1;

    const std::string ownPath = GetFirstTagText(xml, "path");
    if (!ownPath.empty()) {
        const std::filesystem::path resolved = ResolveFlightGearPath(xmlPath, context.aircraftRoot, ownPath);
        if (Lower(resolved.extension().string()) == ".ac") {
            if (!LoadAcFile(resolved, transform, context, mesh, error)) {
                return false;
            }
        }
    }

    for (const XmlModelRef& ref : ParseModelRefs(xml)) {
        if (!ShouldIncludeModelPath(ref.path, options)) {
            context.stats.skippedModels += 1;
            continue;
        }
        const std::filesystem::path resolved = ResolveFlightGearPath(xmlPath, context.aircraftRoot, ref.path);
        const Matrix4 childTransform = Multiply(transform, OffsetMatrix(ref.offset));
        const std::string ext = Lower(resolved.extension().string());
        if (ext == ".xml") {
            if (!LoadModelXmlRecursive(resolved, childTransform, context, options, mesh, error)) {
                return false;
            }
        } else if (ext == ".ac") {
            if (!LoadAcFile(resolved, childTransform, context, mesh, error)) {
                return false;
            }
        }
    }

    return true;
}

} // namespace

bool LoadFlightGearAircraftMesh(
    const std::filesystem::path& modelXmlPath,
    MeshData& outMesh,
    GlbMaterialTexture& outTexture,
    FlightGearLoadStats& outStats,
    const FlightGearLoadOptions& options,
    std::string& error) {
    outMesh = {};
    outTexture = {};
    outStats = {};

    if (!std::filesystem::exists(modelXmlPath)) {
        error = "FlightGear model XML not found: " + modelXmlPath.string();
        return false;
    }

    ParseContext context{};
    context.aircraftRoot = modelXmlPath.parent_path().parent_path();
    if (!LoadModelXmlRecursive(modelXmlPath, Identity(), context, options, outMesh, error)) {
        outStats = context.stats;
        return false;
    }

    if (!outMesh.IsValid()) {
        outStats = context.stats;
        error = "FlightGear aircraft import produced an empty mesh";
        return false;
    }

    outTexture.rgbaPixels = {255, 255, 255, 255};
    outTexture.width = 1;
    outTexture.height = 1;
    outStats = context.stats;
    return true;
}

} // namespace flight
