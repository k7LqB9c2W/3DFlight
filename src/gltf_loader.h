#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "mesh.h"

namespace flight {

struct GlbMaterialTexture {
    std::vector<uint8_t> rgbaPixels;
    uint32_t width = 0;
    uint32_t height = 0;

    [[nodiscard]] bool IsValid() const {
        return width > 0 && height > 0 && rgbaPixels.size() == static_cast<size_t>(width) * static_cast<size_t>(height) * 4ull;
    }
};

bool LoadGlbMesh(const std::filesystem::path& glbPath, MeshData& outMesh, std::string& error);
bool LoadGlbMesh(
    const std::filesystem::path& glbPath,
    MeshData& outMesh,
    GlbMaterialTexture& outBaseColorTexture,
    std::string& error);

} // namespace flight
