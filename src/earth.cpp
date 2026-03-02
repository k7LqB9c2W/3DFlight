#include "earth.h"

#include <cmath>

namespace flight {

MeshData GenerateEarthSphere(float radiusMeters, uint32_t longitudeSegments, uint32_t latitudeSegments) {
    MeshData mesh;
    if (longitudeSegments < 3 || latitudeSegments < 2) {
        return mesh;
    }

    const uint32_t vertsX = longitudeSegments + 1;
    const uint32_t vertsY = latitudeSegments + 1;
    mesh.vertices.reserve(static_cast<size_t>(vertsX) * static_cast<size_t>(vertsY));
    mesh.indices.reserve(static_cast<size_t>(longitudeSegments) * static_cast<size_t>(latitudeSegments) * 6);

    constexpr double kPi = 3.14159265358979323846;

    for (uint32_t y = 0; y <= latitudeSegments; ++y) {
        const double v = static_cast<double>(y) / static_cast<double>(latitudeSegments);
        const double lat = (0.5 - v) * kPi;
        const double cosLat = std::cos(lat);
        const double sinLat = std::sin(lat);

        for (uint32_t x = 0; x <= longitudeSegments; ++x) {
            const double u = static_cast<double>(x) / static_cast<double>(longitudeSegments);
            const double lon = (u * 2.0 - 1.0) * kPi;
            const double cosLon = std::cos(lon);
            const double sinLon = std::sin(lon);

            const float nx = static_cast<float>(cosLat * cosLon);
            const float ny = static_cast<float>(sinLat);
            const float nz = static_cast<float>(cosLat * sinLon);

            Vertex vtx{};
            vtx.position = {nx * radiusMeters, ny * radiusMeters, nz * radiusMeters};
            vtx.normal = {nx, ny, nz};
            vtx.uv = {static_cast<float>(u), static_cast<float>(v)};
            mesh.vertices.push_back(vtx);
        }
    }

    for (uint32_t y = 0; y < latitudeSegments; ++y) {
        for (uint32_t x = 0; x < longitudeSegments; ++x) {
            const uint32_t i0 = y * vertsX + x;
            const uint32_t i1 = i0 + 1;
            const uint32_t i2 = i0 + vertsX;
            const uint32_t i3 = i2 + 1;

            mesh.indices.push_back(i0);
            mesh.indices.push_back(i2);
            mesh.indices.push_back(i1);

            mesh.indices.push_back(i1);
            mesh.indices.push_back(i2);
            mesh.indices.push_back(i3);
        }
    }

    return mesh;
}

} // namespace flight
