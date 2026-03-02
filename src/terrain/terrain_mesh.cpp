#include "terrain/terrain_mesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace flight::terrain {
namespace {

struct TmpVertex {
    Double3 local{};
    Double3 normal{};
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

void OffsetLatLonGreatCircle(
    double centerLatDeg,
    double centerLonDeg,
    double northMeters,
    double eastMeters,
    double& outLatDeg,
    double& outLonDeg) {
    const double distance = std::sqrt(northMeters * northMeters + eastMeters * eastMeters);
    if (distance <= 1e-6) {
        outLatDeg = std::clamp(centerLatDeg, -89.999, 89.999);
        outLonDeg = centerLonDeg;
        NormalizeLon(outLonDeg);
        return;
    }

    const double lat1 = DegToRad(centerLatDeg);
    const double lon1 = DegToRad(centerLonDeg);
    const double bearing = std::atan2(eastMeters, northMeters); // from north, east-positive.
    const double delta = distance / kEarthRadiusMeters;

    const double sinLat1 = std::sin(lat1);
    const double cosLat1 = std::cos(lat1);
    const double sinDelta = std::sin(delta);
    const double cosDelta = std::cos(delta);

    const double sinLat2 = std::clamp(sinLat1 * cosDelta + cosLat1 * sinDelta * std::cos(bearing), -1.0, 1.0);
    const double lat2 = std::asin(sinLat2);

    const double y = std::sin(bearing) * sinDelta * cosLat1;
    const double x = cosDelta - sinLat1 * sinLat2;
    const double lon2 = lon1 + std::atan2(y, x);

    outLatDeg = std::clamp(RadToDeg(lat2), -89.999, 89.999);
    outLonDeg = RadToDeg(lon2);
    NormalizeLon(outLonDeg);
}

TmpVertex SampleTerrainVertex(
    TerrainSystem& terrainSystem,
    double centerLatDeg,
    double centerLonDeg,
    double thetaRad,
    double radiusMeters,
    double skirtDepthMeters,
    double verticalExaggeration,
    const Double3& anchorEcef) {
    const double eastMeters = std::cos(thetaRad) * radiusMeters;
    const double northMeters = std::sin(thetaRad) * radiusMeters;

    double latDeg = centerLatDeg;
    double lonDeg = centerLonDeg;
    OffsetLatLonGreatCircle(centerLatDeg, centerLonDeg, northMeters, eastMeters, latDeg, lonDeg);

    const double rawHeight = terrainSystem.SampleHeightMeters(latDeg, lonDeg);
    const double clampedHeightMeters = std::max(rawHeight, 0.0);
    const double visualHeight = clampedHeightMeters * std::max(verticalExaggeration, 0.01) - skirtDepthMeters;
    const Double3 ecef = GeodeticToEcef(DegToRad(latDeg), DegToRad(lonDeg), visualHeight);

    TmpVertex v{};
    v.local = ecef - anchorEcef;
    v.normal = Normalize(ecef);
    v.clampedHeight = clampedHeightMeters;
    v.rawHeight = rawHeight;
    return v;
}

void AppendRingMesh(
    TerrainSystem& terrainSystem,
    double centerLatDeg,
    double centerLonDeg,
    const Double3& anchorEcef,
    double innerRadiusMeters,
    double outerRadiusMeters,
    int radialSegments,
    int angularSegments,
    double skirtDepthMeters,
    double verticalExaggeration,
    bool includeInnerSkirt,
    bool includeOuterSkirt,
    MeshData& mesh,
    double& minRaw,
    double& maxRaw) {
    struct RowDef {
        double radius = 0.0;
        double skirtDepth = 0.0;
        bool isSkirt = false;
    };

    std::vector<RowDef> rows;
    rows.reserve(static_cast<size_t>(radialSegments) + 3);
    if (includeInnerSkirt) {
        rows.push_back(RowDef{innerRadiusMeters, skirtDepthMeters, true});
    }
    for (int r = 0; r <= radialSegments; ++r) {
        const double t = static_cast<double>(r) / static_cast<double>(std::max(radialSegments, 1));
        rows.push_back(RowDef{
            std::lerp(innerRadiusMeters, outerRadiusMeters, t),
            0.0,
            false,
        });
    }
    if (includeOuterSkirt) {
        rows.push_back(RowDef{outerRadiusMeters, skirtDepthMeters, true});
    }

    const uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());
    const uint32_t cols = static_cast<uint32_t>(angularSegments + 1);

    mesh.vertices.reserve(mesh.vertices.size() + rows.size() * cols);
    for (const RowDef& row : rows) {
        for (int a = 0; a <= angularSegments; ++a) {
            const double theta = (2.0 * 3.14159265358979323846 * static_cast<double>(a)) / static_cast<double>(angularSegments);
            const TmpVertex sample = SampleTerrainVertex(
                terrainSystem,
                centerLatDeg,
                centerLonDeg,
                theta,
                row.radius,
                row.skirtDepth,
                verticalExaggeration,
                anchorEcef);

            Vertex v{};
            v.position = ToFloat3(sample.local);
            v.normal = ToFloat3(sample.normal);
            v.uv = {
                static_cast<float>(sample.clampedHeight),
                static_cast<float>(sample.rawHeight),
            };
            mesh.vertices.push_back(v);

            if (!row.isSkirt) {
                minRaw = std::min(minRaw, sample.rawHeight);
                maxRaw = std::max(maxRaw, sample.rawHeight);
            }
        }
    }

    for (uint32_t r = 0; r + 1 < static_cast<uint32_t>(rows.size()); ++r) {
        for (uint32_t c = 0; c < static_cast<uint32_t>(angularSegments); ++c) {
            const uint32_t i0 = baseVertex + r * cols + c;
            const uint32_t i1 = i0 + 1;
            const uint32_t i2 = i0 + cols;
            const uint32_t i3 = i2 + 1;

            mesh.indices.push_back(i0);
            mesh.indices.push_back(i2);
            mesh.indices.push_back(i1);

            mesh.indices.push_back(i1);
            mesh.indices.push_back(i2);
            mesh.indices.push_back(i3);
        }
    }
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

    const int baseGrid = std::clamp(settings.gridResolution, 65, 513);
    const double lodDistanceScale = std::clamp(settings.lodDistanceScale, 0.35, 4.0);
    const double lodResolutionScale = std::clamp(settings.lodResolutionScale, 0.35, 2.5);
    const double nearToMidMultiplier = std::clamp(settings.nearToMidMultiplier, 1.4, 4.5);
    const double midToFarMultiplier = std::clamp(settings.midToFarMultiplier, 1.4, 4.5);
    const double verticalExaggeration = std::clamp(settings.verticalExaggeration, 0.25, 4.0);

    const double nearRadiusMeters = std::clamp(settings.patchSizeKm * 500.0 * lodDistanceScale, 12000.0, 220000.0);
    const double ring1RadiusMeters = nearRadiusMeters * nearToMidMultiplier;
    const double ring2RadiusMeters = ring1RadiusMeters * midToFarMultiplier;
    const double requestedFarRadiusMeters = std::max(settings.farFieldRadiusKm * 1000.0 * lodDistanceScale, ring2RadiusMeters * 1.6);
    const double farRadiusMeters = std::clamp(requestedFarRadiusMeters, ring2RadiusMeters * 1.2, 3000000.0);
    const double lodTransitionMeters = std::clamp(nearRadiusMeters * 0.12, 2000.0, 14000.0);
    const double skirtDepthMeters = std::clamp(nearRadiusMeters * 0.025, 120.0, 2200.0);

    const int scaledGrid = static_cast<int>(std::round(static_cast<double>(baseGrid) * lodResolutionScale));
    const int angularSegments = std::clamp(((scaledGrid / 2) * 2), 64, 640);
    const int radial0 = std::clamp(scaledGrid / 3, 20, 220);
    const int radial1 = std::clamp(scaledGrid / 5, 14, 140);
    const int radial2 = std::clamp(scaledGrid / 7, 10, 96);
    const int radial3 = std::clamp(scaledGrid / 10, 8, 72);

    MeshData mesh;
    mesh.vertices.reserve(150000);
    mesh.indices.reserve(500000);

    double minRaw = std::numeric_limits<double>::max();
    double maxRaw = -std::numeric_limits<double>::max();

    AppendRingMesh(
        terrainSystem,
        centerLatDeg,
        centerLonDeg,
        anchorEcef,
        0.0,
        nearRadiusMeters,
        radial0,
        angularSegments,
        skirtDepthMeters,
        verticalExaggeration,
        false,
        true,
        mesh,
        minRaw,
        maxRaw);

    AppendRingMesh(
        terrainSystem,
        centerLatDeg,
        centerLonDeg,
        anchorEcef,
        nearRadiusMeters + 50.0,
        ring1RadiusMeters,
        radial1,
        angularSegments,
        skirtDepthMeters,
        verticalExaggeration,
        true,
        true,
        mesh,
        minRaw,
        maxRaw);

    AppendRingMesh(
        terrainSystem,
        centerLatDeg,
        centerLonDeg,
        anchorEcef,
        ring1RadiusMeters + 100.0,
        ring2RadiusMeters,
        radial2,
        angularSegments,
        skirtDepthMeters,
        verticalExaggeration,
        true,
        true,
        mesh,
        minRaw,
        maxRaw);

    // Far-field fallback shell ring to avoid visible patch edge at horizon.
    AppendRingMesh(
        terrainSystem,
        centerLatDeg,
        centerLonDeg,
        anchorEcef,
        ring2RadiusMeters + 200.0,
        farRadiusMeters,
        radial3,
        angularSegments,
        skirtDepthMeters * 1.5,
        verticalExaggeration,
        true,
        true,
        mesh,
        minRaw,
        maxRaw);

    if (!mesh.IsValid()) {
        error = "Generated terrain clipmap mesh is empty";
        return false;
    }

    const double resolvedMinRaw = (minRaw == std::numeric_limits<double>::max()) ? 0.0 : minRaw;
    const double resolvedMaxRaw = (maxRaw == -std::numeric_limits<double>::max()) ? 0.0 : maxRaw;
    const float colorRangeMax = static_cast<float>(std::clamp(std::max(resolvedMaxRaw, 2000.0), 2000.0, 14000.0));

    outResult.mesh = std::move(mesh);
    outResult.anchorEcef = anchorEcef;
    outResult.centerLatDeg = centerLatDeg;
    outResult.centerLonDeg = centerLonDeg;
    outResult.minHeightRaw = resolvedMinRaw;
    outResult.maxHeightRaw = resolvedMaxRaw;
    outResult.renderParams = {
        static_cast<float>(nearRadiusMeters),
        colorRangeMax,
        static_cast<float>(lodTransitionMeters),
        static_cast<float>(farRadiusMeters),
    };

    return true;
}

} // namespace flight::terrain
