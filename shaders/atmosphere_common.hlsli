static const float kPi = 3.14159265359;
static const float kInvFourPi = 0.07957747154;

cbuffer SceneCB : register(b0)
{
    float4x4 gViewProj;
    float4x4 gInvViewProj;
    float4 gEarthCenterRadius; // xyz center, w Rg (m)
    float4 gSunDirIntensity;   // xyz sun dir, w sun illuminance scale
    float4 gCameraPosTopRadius; // xyz camera position, w Rt (m)
    float4 gCameraUpAndTime; // xyz camera up, w seconds
    float4 gViewportAndAerialDepth; // x width, y height, z aerial depth max (m), w atmosphere enabled
    float4 gAtmosphereFlags; // x multiple scattering enabled, y mie g, z exposure, w reserved
    float4 gRayleighScatteringAndScale; // rgb sigma_s^r, w Hr
    float4 gRayleighAbsorptionPad; // rgb sigma_a^r
    float4 gMieScatteringAndScale; // rgb sigma_s^m, w Hm
    float4 gMieAbsorptionAndG; // rgb sigma_a^m, w g
    float4 gOzoneAbsorptionAndCenter; // rgb sigma_a^o, w ozone center height
    float4 gOzoneHalfWidthAtmosphereHeightSunRadius; // x ozone half-width, y atmosphere height, z sun radius
};

float GroundRadiusMeters()
{
    return gEarthCenterRadius.w;
}

float TopRadiusMeters()
{
    return gCameraPosTopRadius.w;
}

float3 PlanetCenter()
{
    return gEarthCenterRadius.xyz;
}

float3 CameraPosition()
{
    return gCameraPosTopRadius.xyz;
}

float AtmosphereHeightMeters()
{
    return gOzoneHalfWidthAtmosphereHeightSunRadius.y;
}

float AltitudeMeters(float3 worldPos)
{
    return max(0.0, length(worldPos - PlanetCenter()) - GroundRadiusMeters());
}

float DensityRayleigh(float altitudeMeters)
{
    // Paper density profile d_r(h) = exp(-h / H_r).
    return exp(-altitudeMeters / max(gRayleighScatteringAndScale.w, 1.0));
}

float DensityMie(float altitudeMeters)
{
    // Paper density profile d_m(h) = exp(-h / H_m).
    return exp(-altitudeMeters / max(gMieScatteringAndScale.w, 1.0));
}

float DensityOzone(float altitudeMeters)
{
    // Paper ozone tent profile d_o(h) = max(0, 1 - |h-c|/w).
    float center = gOzoneAbsorptionAndCenter.w;
    float halfWidth = max(gOzoneHalfWidthAtmosphereHeightSunRadius.x, 1.0);
    return max(0.0, 1.0 - abs(altitudeMeters - center) / halfWidth);
}

float3 RayleighScatteringCoefficient(float altitudeMeters)
{
    return gRayleighScatteringAndScale.rgb * DensityRayleigh(altitudeMeters);
}

float3 MieScatteringCoefficient(float altitudeMeters)
{
    return gMieScatteringAndScale.rgb * DensityMie(altitudeMeters);
}

float3 ScatteringCoefficient(float altitudeMeters)
{
    return RayleighScatteringCoefficient(altitudeMeters) + MieScatteringCoefficient(altitudeMeters);
}

float3 ExtinctionCoefficient(float altitudeMeters)
{
    const float dr = DensityRayleigh(altitudeMeters);
    const float dm = DensityMie(altitudeMeters);
    const float dozone = DensityOzone(altitudeMeters);
    const float3 sigmaR = (gRayleighScatteringAndScale.rgb + gRayleighAbsorptionPad.rgb) * dr;
    const float3 sigmaM = (gMieScatteringAndScale.rgb + gMieAbsorptionAndG.rgb) * dm;
    const float3 sigmaO = gOzoneAbsorptionAndCenter.rgb * dozone;
    return sigmaR + sigmaM + sigmaO;
}

// Rayleigh phase (Eq on paper page 3).
float RayleighPhase(float cosTheta)
{
    return (3.0 * (1.0 + cosTheta * cosTheta)) / (16.0 * kPi);
}

// Cornette-Shanks Mie phase (paper page 3).
float MieCornetteShanksPhase(float cosTheta, float g)
{
    float gg = g * g;
    float denom = max(1e-4, pow(1.0 + gg - 2.0 * g * cosTheta, 1.5));
    return (3.0 / (8.0 * kPi)) * ((1.0 - gg) * (1.0 + cosTheta * cosTheta)) / ((2.0 + gg) * denom);
}

float IsotropicPhase()
{
    return kInvFourPi;
}

void BuildSkyBasis(float3 cameraUp, float3 sunDir, out float3 north, out float3 east)
{
    const float3 up = normalize(cameraUp);
    float3 projectedSun = sunDir - up * dot(sunDir, up);
    if (length(projectedSun) < 1e-5)
    {
        projectedSun = normalize(cross(up, float3(1.0, 0.0, 0.0)));
    }
    north = normalize(projectedSun);
    east = normalize(cross(up, north));
}

// Paper non-linear latitude mapping: v = 0.5 + 0.5*sign(l)*sqrt(|l|/(pi/2)).
float EncodeSkyLatitude(float latitudeRad)
{
    const float normalized = abs(latitudeRad) / (0.5 * kPi);
    const float signedSqrt = sign(latitudeRad) * sqrt(saturate(normalized));
    return 0.5 + 0.5 * signedSqrt;
}

float DecodeSkyLatitude(float v)
{
    const float s = 2.0 * v - 1.0;
    const float absS = abs(s);
    return sign(s) * (absS * absS) * (0.5 * kPi);
}

float2 DirectionToSkyViewUv(float3 dir, float3 cameraUp, float3 sunDir)
{
    float3 north;
    float3 east;
    BuildSkyBasis(cameraUp, sunDir, north, east);

    const float3 up = normalize(cameraUp);
    const float latitude = asin(clamp(dot(normalize(dir), up), -1.0, 1.0));
    const float longitude = atan2(dot(dir, east), dot(dir, north));
    const float u = frac(longitude / (2.0 * kPi) + 0.5);
    const float v = EncodeSkyLatitude(latitude);
    return float2(u, v);
}

float3 SkyViewUvToDirection(float2 uv, float3 cameraUp, float3 sunDir)
{
    float3 north;
    float3 east;
    BuildSkyBasis(cameraUp, sunDir, north, east);
    const float3 up = normalize(cameraUp);

    const float longitude = (uv.x - 0.5) * (2.0 * kPi);
    const float latitude = DecodeSkyLatitude(uv.y);
    const float cLat = cos(latitude);
    const float sLat = sin(latitude);
    return normalize(north * (cLat * cos(longitude)) + east * (cLat * sin(longitude)) + up * sLat);
}

float2 AtmosphereLutUvFromHeightMu(float altitudeMeters, float mu)
{
    const float u = saturate(mu * 0.5 + 0.5);
    const float v = saturate(altitudeMeters / max(AtmosphereHeightMeters(), 1.0));
    return float2(u, v);
}

float2 MultipleScatteringUv(float altitudeMeters, float muSun)
{
    // Paper LUT parameterization: u = 0.5 + 0.5*cos(theta_s), v = normalized altitude.
    return float2(saturate(0.5 + 0.5 * muSun), saturate(altitudeMeters / max(AtmosphereHeightMeters(), 1.0)));
}

float3 ApplyAerialPerspectiveToColor(
    Texture3D<float4> aerialLut,
    SamplerState clampSampler,
    float2 pixelPos,
    float viewDistanceMeters,
    float3 baseColor)
{
    if (gViewportAndAerialDepth.w < 0.5)
    {
        return baseColor;
    }

    const float2 viewportSize = max(gViewportAndAerialDepth.xy, float2(1.0, 1.0));
    const float2 uv = saturate(pixelPos / viewportSize);
    const float depthT = saturate(viewDistanceMeters / max(gViewportAndAerialDepth.z, 1.0));
    const float4 ap = aerialLut.SampleLevel(clampSampler, float3(uv, depthT), 0.0);
    return baseColor * ap.a + ap.rgb;
}
