#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <DirectXMath.h>

#include "gltf_loader.h"
#include "mesh.h"

namespace flight {

struct FlightGearLoadOptions {
    bool includeCockpit = true;
    bool includePassengerSeats = true;
    bool includeLights = true;
};

struct FlightGearLoadStats {
    uint32_t xmlFilesLoaded = 0;
    uint32_t acFilesLoaded = 0;
    uint32_t texturesLoaded = 0;
    uint32_t texturesMissing = 0;
    uint32_t skippedModels = 0;
};

struct FlightGearMeshPart {
    MeshData mesh;
    GlbMaterialTexture texture;
    std::string textureName;
    std::string objectName;
    std::vector<std::string> objectAliases;
    bool transparent = false;
    bool emissive = false;
    bool doubleSided = false;
    float alphaCutoff = 0.0f;
};

struct FlightGearAnimationEntry {
    float input = 0.0f;
    float output = 0.0f;
};

struct FlightGearAnimation {
    enum class Type {
        Select,
        Rotate,
        Translate,
        Spin,
        Scale,
        DistScale,
        Billboard,
    };

    Type type = Type::Rotate;
    std::vector<std::string> objectNames;
    std::string property;
    std::string conditionProperty;
    bool conditionInvert = false;
    float factor = 1.0f;
    float offset = 0.0f;
    float minValue = -3.402823466e+38f;
    float maxValue = 3.402823466e+38f;
    DirectX::XMFLOAT3 center{0.0f, 0.0f, 0.0f};
    DirectX::XMFLOAT3 axis{0.0f, 1.0f, 0.0f};
    DirectX::XMFLOAT3 scaleFactor{1.0f, 1.0f, 1.0f};
    DirectX::XMFLOAT3 scaleOffset{0.0f, 0.0f, 0.0f};
    DirectX::XMFLOAT3 scaleMin{-3.402823466e+38f, -3.402823466e+38f, -3.402823466e+38f};
    DirectX::XMFLOAT3 scaleMax{3.402823466e+38f, 3.402823466e+38f, 3.402823466e+38f};
    bool sphericalBillboard = true;
    std::vector<FlightGearAnimationEntry> interpolation;
};

bool LoadFlightGearAircraftMesh(
    const std::filesystem::path& modelXmlPath,
    MeshData& outMesh,
    GlbMaterialTexture& outTexture,
    FlightGearLoadStats& outStats,
    const FlightGearLoadOptions& options,
    std::string& error);

bool LoadFlightGearAircraftParts(
    const std::filesystem::path& modelXmlPath,
    std::vector<FlightGearMeshPart>& outParts,
    std::vector<FlightGearAnimation>& outAnimations,
    FlightGearLoadStats& outStats,
    const FlightGearLoadOptions& options,
    std::string& error);

} // namespace flight
