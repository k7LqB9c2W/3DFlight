#pragma once

#include <filesystem>
#include <string>

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

bool LoadFlightGearAircraftMesh(
    const std::filesystem::path& modelXmlPath,
    MeshData& outMesh,
    GlbMaterialTexture& outTexture,
    FlightGearLoadStats& outStats,
    const FlightGearLoadOptions& options,
    std::string& error);

} // namespace flight
