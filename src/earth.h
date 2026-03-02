#pragma once

#include "mesh.h"

namespace flight {

MeshData GenerateEarthSphere(float radiusMeters, uint32_t longitudeSegments = 96, uint32_t latitudeSegments = 48);

} // namespace flight
