#pragma once

#include <filesystem>
#include <string>

#include "mesh.h"

namespace flight {

bool LoadGlbMesh(const std::filesystem::path& glbPath, MeshData& outMesh, std::string& error);

} // namespace flight
