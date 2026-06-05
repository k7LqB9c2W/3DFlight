#pragma once

#include <filesystem>
#include <string>
#include <vector>

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
    FlightGearLoadStats& outStats,
    const FlightGearLoadOptions& options,
    std::string& error);

} // namespace flight
