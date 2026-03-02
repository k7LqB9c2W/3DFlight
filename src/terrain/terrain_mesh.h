#pragma once

#include <string>

#include "mesh.h"
#include "sim.h"
#include "terrain/terrain_system.h"

namespace flight::terrain {

struct TerrainPatchSettings {
    double patchSizeKm = 80.0;
    int gridResolution = 257;
};

struct TerrainPatchBuildResult {
    MeshData mesh;
    Double3 anchorEcef{};
    double centerLatDeg = 0.0;
    double centerLonDeg = 0.0;
    double minHeightRaw = 0.0;
    double maxHeightRaw = 0.0;
};

bool BuildTerrainPatch(
    TerrainSystem& terrainSystem,
    double centerLatDeg,
    double centerLonDeg,
    const Double3& anchorEcef,
    const TerrainPatchSettings& settings,
    TerrainPatchBuildResult& outResult,
    std::string& error);

} // namespace flight::terrain
