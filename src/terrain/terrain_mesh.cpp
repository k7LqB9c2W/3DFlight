#include "terrain/terrain_mesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace flight::terrain {
namespace {

struct TmpVertex {
    Double3 local;
    double clampedHeight = 0.0;
    double rawHeight = 0.0;
};

void NormalizeLon(double& lonDeg) {
    while (lonDeg >= 180.0) {
        lonDeg -= 360.0;
    }
    while (lonDeg < -180.0) {
        lonDeg += 360.0;
    }
}

DirectX::XMFLOAT3 ToFloat3(const Double3& v) {
    return {
        static_cast<float>(v.x),
        static_cast<float>(v.y),
        static_cast<float>(v.z),
    };
}

} // namespace

bool BuildTerrainPatch(
    TerrainSystem& terrainSystem,
    double centerLatDeg,
    double centerLonDeg,
    const Double3& anchorEcef,
    const TerrainPatchSettings& settings,
    TerrainPatchBuildResult& outResult,
    std::string& error) {
    outResult = {};

    const int n = std::clamp(settings.gridResolution, 33, 513);
    if (n < 2) {
        error = "Terrain grid resolution must be >= 2";
        return false;
    }

    const double patchMeters = std::clamp(settings.patchSizeKm, 10.0, 300.0) * 1000.0;
    const double half = patchMeters * 0.5;
    const double step = patchMeters / static_cast<double>(n - 1);
    const double centerLatRad = DegToRad(centerLatDeg);

    std::vector<TmpVertex> tmp(static_cast<size_t>(n) * static_cast<size_t>(n));

    double minRaw = std::numeric_limits<double>::max();
    double maxRaw = -std::numeric_limits<double>::max();

    for (int y = 0; y < n; ++y) {
        const double northMeters = half - static_cast<double>(y) * step;

        for (int x = 0; x < n; ++x) {
            const double eastMeters = -half + static_cast<double>(x) * step;

            const double dLatDeg = RadToDeg(northMeters / kEarthRadiusMeters);
            const double cosLat = std::max(0.01, std::cos(centerLatRad));
            const double dLonDeg = RadToDeg(eastMeters / (kEarthRadiusMeters * cosLat));

            double latDeg = std::clamp(centerLatDeg + dLatDeg, -89.999, 89.999);
            double lonDeg = centerLonDeg + dLonDeg;
            NormalizeLon(lonDeg);

            const double rawHeight = terrainSystem.SampleHeightMeters(latDeg, lonDeg);
            const double clampedHeight = std::max(rawHeight, 0.0);
            const Double3 ecef = GeodeticToEcef(DegToRad(latDeg), DegToRad(lonDeg), clampedHeight);

            TmpVertex tv{};
            tv.local = ecef - anchorEcef;
            tv.clampedHeight = clampedHeight;
            tv.rawHeight = rawHeight;
            tmp[static_cast<size_t>(y) * static_cast<size_t>(n) + static_cast<size_t>(x)] = tv;

            minRaw = std::min(minRaw, rawHeight);
            maxRaw = std::max(maxRaw, rawHeight);
        }
    }

    outResult.mesh.vertices.resize(static_cast<size_t>(n) * static_cast<size_t>(n));
    outResult.mesh.indices.reserve(static_cast<size_t>(n - 1) * static_cast<size_t>(n - 1) * 6);

    auto at = [&](int x, int y) -> const TmpVertex& {
        return tmp[static_cast<size_t>(y) * static_cast<size_t>(n) + static_cast<size_t>(x)];
    };

    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const TmpVertex& c = at(x, y);

            const int xl = std::max(0, x - 1);
            const int xr = std::min(n - 1, x + 1);
            const int yd = std::max(0, y - 1);
            const int yu = std::min(n - 1, y + 1);

            const Double3 pL = at(xl, y).local;
            const Double3 pR = at(xr, y).local;
            const Double3 pD = at(x, yd).local;
            const Double3 pU = at(x, yu).local;

            const Double3 tangentE = pR - pL;
            const Double3 tangentN = pD - pU;
            Double3 normal = Normalize(Cross(tangentE, tangentN));
            if (Length(normal) < 1e-8) {
                normal = Normalize(c.local);
            }

            Vertex v{};
            v.position = ToFloat3(c.local);
            v.normal = ToFloat3(normal);
            v.uv = {
                static_cast<float>(c.clampedHeight),
                static_cast<float>(c.rawHeight),
            };
            outResult.mesh.vertices[static_cast<size_t>(y) * static_cast<size_t>(n) + static_cast<size_t>(x)] = v;
        }
    }

    for (int y = 0; y < n - 1; ++y) {
        for (int x = 0; x < n - 1; ++x) {
            const uint32_t i0 = static_cast<uint32_t>(y * n + x);
            const uint32_t i1 = i0 + 1;
            const uint32_t i2 = i0 + static_cast<uint32_t>(n);
            const uint32_t i3 = i2 + 1;

            outResult.mesh.indices.push_back(i0);
            outResult.mesh.indices.push_back(i2);
            outResult.mesh.indices.push_back(i1);

            outResult.mesh.indices.push_back(i1);
            outResult.mesh.indices.push_back(i2);
            outResult.mesh.indices.push_back(i3);
        }
    }

    outResult.anchorEcef = anchorEcef;
    outResult.centerLatDeg = centerLatDeg;
    outResult.centerLonDeg = centerLonDeg;
    outResult.minHeightRaw = (minRaw == std::numeric_limits<double>::max()) ? 0.0 : minRaw;
    outResult.maxHeightRaw = (maxRaw == -std::numeric_limits<double>::max()) ? 0.0 : maxRaw;

    if (!outResult.mesh.IsValid()) {
        error = "Generated terrain patch mesh is empty";
        return false;
    }

    return true;
}

} // namespace flight::terrain
