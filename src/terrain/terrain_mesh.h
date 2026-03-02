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
    // x = near LOD radius (m), y = terrain color range max height (m),
    // z = LOD transition width (m), w = far LOD radius (m).
    DirectX::XMFLOAT4 renderParams{40000.0f, 6000.0f, 5000.0f, 900000.0f};
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
