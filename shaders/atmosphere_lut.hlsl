#include "atmosphere_common.hlsli"

Texture2D<float> gLandmask : register(t0);
TextureCube<float4> gSkybox : register(t1);
Texture2D<float4> gTransmittanceLut : register(t2);
Texture2D<float4> gSkyViewLut : register(t3);
Texture2D<float4> gMultipleScatteringLut : register(t4);
Texture3D<float4> gAerialPerspectiveLut : register(t5);

RWTexture2D<float4> gTransmittanceLutOut : register(u0);
RWTexture2D<float4> gSkyViewLutOut : register(u1);
RWTexture2D<float4> gMultipleScatteringLutOut : register(u2);
RWTexture3D<float4> gAerialPerspectiveLutOut : register(u3);

SamplerState gWrapSampler : register(s0);
SamplerState gClampSampler : register(s1);

static const uint kTransmittanceSampleCount = 40;
static const uint kSkySampleCount = 32;
static const uint kSecondOrderRaySampleCount = 12;
static const uint kAerialSampleCount = 16;
static const uint kSphereIntegralDirectionCount = 64;

bool RaySphereIntersection(float3 rayOrigin, float3 rayDir, float3 center, float radius, out float tNear, out float tFar)
{
    float3 oc = rayOrigin - center;
    float b = dot(oc, rayDir);
    float c = dot(oc, oc) - radius * radius;
    float d = b * b - c;
    if (d < 0.0)
    {
        tNear = 0.0;
        tFar = 0.0;
        return false;
    }
    float s = sqrt(d);
    tNear = -b - s;
    tFar = -b + s;
    return true;
}

float FirstPositiveDistance(float a, float b)
{
    float t = 1e30;
    if (a > 0.0)
    {
        t = min(t, a);
    }
    if (b > 0.0)
    {
        t = min(t, b);
    }
    return (t < 1e29) ? t : -1.0;
}

float DistanceToAtmosphereBoundary(float3 rayOrigin, float3 rayDir, out bool hitsGroundFirst)
{
    float tAtmoNear = 0.0;
    float tAtmoFar = 0.0;
    if (!RaySphereIntersection(rayOrigin, rayDir, PlanetCenter(), TopRadiusMeters(), tAtmoNear, tAtmoFar))
    {
        hitsGroundFirst = false;
        return 0.0;
    }

    float tTop = FirstPositiveDistance(tAtmoNear, tAtmoFar);
    if (tTop <= 0.0)
    {
        hitsGroundFirst = false;
        return 0.0;
    }

    hitsGroundFirst = false;
    float tGroundNear = 0.0;
    float tGroundFar = 0.0;
    if (RaySphereIntersection(rayOrigin, rayDir, PlanetCenter(), GroundRadiusMeters(), tGroundNear, tGroundFar))
    {
        float tGround = FirstPositiveDistance(tGroundNear, tGroundFar);
        if (tGround > 0.0 && tGround < tTop)
        {
            hitsGroundFirst = true;
            return tGround;
        }
    }

    return tTop;
}

bool IsSunOccludedByPlanet(float3 worldPos, float3 sunDir)
{
    bool hitsGround = false;
    float3 origin = worldPos + sunDir * 8.0;
    DistanceToAtmosphereBoundary(origin, sunDir, hitsGround);
    return hitsGround;
}

float3 IntegrateTransmittance(float3 x0, float3 x1, uint sampleCount)
{
    float3 delta = x1 - x0;
    float segmentLength = length(delta);
    if (segmentLength <= 1e-4)
    {
        return float3(1.0, 1.0, 1.0);
    }

    float3 dir = delta / segmentLength;
    float dt = segmentLength / max(sampleCount, 1);
    float3 opticalDepth = 0.0;
    [loop]
    for (uint i = 0; i < sampleCount; ++i)
    {
        float t = (i + 0.5) * dt;
        float3 samplePos = x0 + dir * t;
        opticalDepth += ExtinctionCoefficient(AltitudeMeters(samplePos)) * dt;
    }
    return exp(-opticalDepth);
}

float3 SampleTransmittanceToTop(float3 worldPos, float3 rayDir)
{
    float3 up = normalize(worldPos - PlanetCenter());
    float mu = dot(up, rayDir);
    float2 uv = AtmosphereLutUvFromHeightMu(AltitudeMeters(worldPos), mu);
    return gTransmittanceLut.SampleLevel(gClampSampler, uv, 0).rgb;
}

struct ScatteringIntegrationResult
{
    float3 inscatter;
    float3 transmittance;
};

// Eq (3), Eq (4) and Eq (11): single scattering plus multiple scattering injection.
ScatteringIntegrationResult IntegrateViewScattering(float3 rayOrigin, float3 rayDir, float maxDistance, uint sampleCount, bool includeMultipleScattering)
{
    ScatteringIntegrationResult r;
    r.inscatter = 0.0;
    r.transmittance = 1.0;

    if (maxDistance <= 1e-4)
    {
        return r;
    }

    float dt = maxDistance / max(sampleCount, 1);
    float3 opticalDepth = 0.0;
    float3 sunDir = normalize(gSunDirIntensity.xyz);

    [loop]
    for (uint i = 0; i < sampleCount; ++i)
    {
        float t = (i + 0.5) * dt;
        float3 samplePos = rayOrigin + rayDir * t;
        float altitude = AltitudeMeters(samplePos);

        float3 sigmaT = ExtinctionCoefficient(altitude);
        opticalDepth += sigmaT * dt;
        float3 tView = exp(-opticalDepth);

        float3 up = normalize(samplePos - PlanetCenter());
        float mu = dot(rayDir, sunDir);
        float muSun = dot(up, sunDir);

        float3 tSun = IsSunOccludedByPlanet(samplePos, sunDir)
            ? float3(0.0, 0.0, 0.0)
            : SampleTransmittanceToTop(samplePos, sunDir);

        float phaseR = RayleighPhase(mu);
        float phaseM = MieCornetteShanksPhase(mu, gAtmosphereFlags.y);
        float3 phaseScattering =
            RayleighScatteringCoefficient(altitude) * phaseR +
            MieScatteringCoefficient(altitude) * phaseM;

        if (includeMultipleScattering && gAtmosphereFlags.x > 0.5)
        {
            float2 uvMs = MultipleScatteringUv(altitude, muSun);
            float3 psiMs = gMultipleScatteringLut.SampleLevel(gClampSampler, uvMs, 0).rgb;
            // Eq (11) injection term.
            phaseScattering += ScatteringCoefficient(altitude) * psiMs;
        }

        r.inscatter += tView * tSun * phaseScattering * dt * gSunDirIntensity.w;
    }

    r.transmittance = exp(-opticalDepth);
    return r;
}

float3 UniformSphereDirection(uint index, uint count)
{
    float u = (index + 0.5) / count;
    float v = frac((index + 0.5) * 0.61803398875);
    float cosTheta = 1.0 - 2.0 * u;
    float sinTheta = sqrt(saturate(1.0 - cosTheta * cosTheta));
    float phi = 2.0 * kPi * v;
    return float3(cos(phi) * sinTheta, cosTheta, sin(phi) * sinTheta);
}

// Eq (6) and Eq (8) approximations along one direction.
void IntegrateSecondOrderTerms(float3 rayOrigin, float3 rayDir, float3 sunDir, float maxDistance, out float3 outLPrime, out float3 outLf)
{
    outLPrime = 0.0;
    outLf = 0.0;
    if (maxDistance <= 1e-4)
    {
        return;
    }

    float dt = maxDistance / kSecondOrderRaySampleCount;
    float3 opticalDepth = 0.0;

    [loop]
    for (uint i = 0; i < kSecondOrderRaySampleCount; ++i)
    {
        float t = (i + 0.5) * dt;
        float3 samplePos = rayOrigin + rayDir * t;
        float altitude = AltitudeMeters(samplePos);
        float3 sigmaT = ExtinctionCoefficient(altitude);
        float3 sigmaS = ScatteringCoefficient(altitude);

        opticalDepth += sigmaT * dt;
        float3 tView = exp(-opticalDepth);
        float3 tSun = IsSunOccludedByPlanet(samplePos, sunDir)
            ? float3(0.0, 0.0, 0.0)
            : SampleTransmittanceToTop(samplePos, sunDir);

        outLPrime += tView * sigmaS * tSun * IsotropicPhase() * dt;
        outLf += tView * sigmaS * dt;
    }
}

float3 ReconstructViewDirection(float2 uv)
{
    float2 ndc = float2(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    float4 farPos = mul(float4(ndc, 1.0, 1.0), gInvViewProj);
    farPos.xyz /= max(farPos.w, 1e-5);
    return normalize(farPos.xyz - CameraPosition());
}

[numthreads(8, 8, 1)]
void CSGenerateTransmittance(uint3 id : SV_DispatchThreadID)
{
    uint width = 0;
    uint height = 0;
    gTransmittanceLutOut.GetDimensions(width, height);
    if (id.x >= width || id.y >= height)
    {
        return;
    }

    float2 uv = (float2(id.xy) + 0.5) / float2(width, height);
    float mu = uv.x * 2.0 - 1.0;
    float altitude = uv.y * AtmosphereHeightMeters();

    float3 origin = PlanetCenter() + float3(0.0, GroundRadiusMeters() + altitude + 1.0, 0.0);
    float3 dir = normalize(float3(sqrt(saturate(1.0 - mu * mu)), mu, 0.0));

    bool hitsGround = false;
    float tMax = DistanceToAtmosphereBoundary(origin, dir, hitsGround);
    float3 transmittance = IntegrateTransmittance(origin, origin + dir * tMax, kTransmittanceSampleCount);
    gTransmittanceLutOut[id.xy] = float4(transmittance, 1.0);
}

[numthreads(8, 8, 1)]
void CSGenerateMultipleScattering(uint3 id : SV_DispatchThreadID)
{
    uint width = 0;
    uint height = 0;
    gMultipleScatteringLutOut.GetDimensions(width, height);
    if (id.x >= width || id.y >= height)
    {
        return;
    }

    float2 uv = (float2(id.xy) + 0.5) / float2(width, height);
    float muSun = uv.x * 2.0 - 1.0;
    float altitude = uv.y * AtmosphereHeightMeters();

    float3 x_s = PlanetCenter() + float3(0.0, GroundRadiusMeters() + altitude + 1.0, 0.0);
    float3 sunDir = normalize(float3(sqrt(saturate(1.0 - muSun * muSun)), muSun, 0.0));

    float3 L2ndOrder = 0.0;
    float3 f_ms = 0.0;
    float dOmega = (4.0 * kPi) / kSphereIntegralDirectionCount;

    [loop]
    for (uint i = 0; i < kSphereIntegralDirectionCount; ++i)
    {
        float3 omega = UniformSphereDirection(i, kSphereIntegralDirectionCount);
        float3 rayDir = -omega;
        bool hitsGround = false;
        float tMax = DistanceToAtmosphereBoundary(x_s, rayDir, hitsGround);

        float3 LPrime = 0.0;
        float3 Lf = 0.0;
        IntegrateSecondOrderTerms(x_s, rayDir, sunDir, tMax, LPrime, Lf);

        // Eq (5) and Eq (7) sphere integrals.
        L2ndOrder += LPrime * IsotropicPhase() * dOmega;
        f_ms += Lf * IsotropicPhase() * dOmega;
    }

    float3 fClamped = min(f_ms, 0.98);
    float3 F_ms = 1.0 / max(1.0 - fClamped, 0.02);
    // Eq (10)
    float3 psi_ms = L2ndOrder * F_ms;
    gMultipleScatteringLutOut[id.xy] = float4(psi_ms, 1.0);
}

[numthreads(8, 8, 1)]
void CSGenerateSkyView(uint3 id : SV_DispatchThreadID)
{
    uint width = 0;
    uint height = 0;
    gSkyViewLutOut.GetDimensions(width, height);
    if (id.x >= width || id.y >= height)
    {
        return;
    }

    float2 uv = (float2(id.xy) + 0.5) / float2(width, height);
    float3 rayDir = SkyViewUvToDirection(uv, normalize(gCameraUpAndTime.xyz), normalize(gSunDirIntensity.xyz));

    bool hitsGround = false;
    float maxDistance = DistanceToAtmosphereBoundary(CameraPosition(), rayDir, hitsGround);
    ScatteringIntegrationResult result = IntegrateViewScattering(
        CameraPosition(),
        rayDir,
        maxDistance,
        kSkySampleCount,
        true);

    gSkyViewLutOut[id.xy] = float4(result.inscatter, 1.0);
}

[numthreads(4, 4, 4)]
void CSGenerateAerialPerspective(uint3 id : SV_DispatchThreadID)
{
    uint width = 0;
    uint height = 0;
    uint depth = 0;
    gAerialPerspectiveLutOut.GetDimensions(width, height, depth);
    if (id.x >= width || id.y >= height || id.z >= depth)
    {
        return;
    }

    float2 uv = (float2(id.xy) + 0.5) / float2(width, height);
    float depthT = (id.z + 0.5) / depth;
    float targetDistance = depthT * max(gViewportAndAerialDepth.z, 1.0);
    float3 rayDir = ReconstructViewDirection(uv);

    bool hitsGround = false;
    float boundaryDistance = DistanceToAtmosphereBoundary(CameraPosition(), rayDir, hitsGround);
    float maxDistance = min(targetDistance, boundaryDistance);
    ScatteringIntegrationResult result = IntegrateViewScattering(
        CameraPosition(),
        rayDir,
        maxDistance,
        kAerialSampleCount,
        true);

    float transmittanceMean = dot(result.transmittance, float3(0.33333334, 0.33333334, 0.33333334));
    gAerialPerspectiveLutOut[id] = float4(result.inscatter, transmittanceMean);
}
