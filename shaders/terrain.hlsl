#include "atmosphere_common.hlsli"

Texture2D<float> gLandmask : register(t0);
TextureCube<float4> gSkybox : register(t1);
Texture2D<float4> gTransmittanceLut : register(t2);
Texture2D<float4> gSkyViewLut : register(t3);
Texture2D<float4> gMultipleScatteringLut : register(t4);
Texture3D<float4> gAerialPerspectiveLut : register(t5);
Texture2D<float4> gEarthAlbedo : register(t6);
Texture2D<float4> gSatelliteNear : register(t8);
Texture2D<float4> gSatelliteMid : register(t9);
Texture2D<float4> gSatelliteFar : register(t10);
Texture2D<float4> gSatellitePrevNear : register(t11);
Texture2D<float4> gSatellitePrevMid : register(t12);
Texture2D<float4> gSatellitePrevFar : register(t13);
Texture2D<float4> gWorldSatelliteAtlas : register(t19);
Texture2D<uint> gWorldSatellitePageTable : register(t20);
Texture2D<uint2> gWorldSatellitePageKeys : register(t21);

SamplerState gWrapSampler : register(s0);
SamplerState gClampSampler : register(s1);

cbuffer ObjectCB : register(b1)
{
    float4x4 gModel;
    float4 gColorAndFlags;
    float4 gTuning0; // x midRingMul, y farRingMul, z hazeStrength, w hazeAltitudeRangeMeters
    float4 gTuning1; // x contrast, y slopeBoost, z specularStrength, w lodSeamBlendStrength
    float4 gTuning2; // x satelliteEnabled, y satelliteBlend, z fallbackWrapLon, w streamedAnyValid
    float4 gTuning3; // x lonWest, y lonEast, z latSouth, w latNorth (degrees)
    float4 gTuning4; // streamed near  bounds lonWest/lonEast/latSouth/latNorth
    float4 gTuning5; // streamed mid   bounds lonWest/lonEast/latSouth/latNorth
    float4 gTuning6; // streamed far   bounds lonWest/lonEast/latSouth/latNorth
    float4 gTuning7; // x nearValid, y midValid, z farValid, w unused
    float4 gTuning8; // previous streamed near bounds lonWest/lonEast/latSouth/latNorth
    float4 gTuning9; // previous streamed mid  bounds lonWest/lonEast/latSouth/latNorth
    float4 gTuning10; // previous streamed far bounds lonWest/lonEast/latSouth/latNorth
    float4 gTuning11; // x prevNearValid, y prevMidValid, z prevFarValid, w transition [0..1] old->new
    float4 gTuning12; // x terrain layer alpha, y worldLockedEnabled, z worldLockedBlend, w unused
    float4 gTuning13; // x atlasPagesX, y atlasPagesY, z pageTableWidth, w pageTableHeight
    float4 gTuning14; // x nearZoom, y midZoom, z farZoom, w maxPageTableProbeCount
};

struct VSInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float2 uv : TEXCOORD0; // x = clamped height meters, y = raw height meters
};

struct VSOutput
{
    float4 position : SV_Position;
    float3 worldPos : TEXCOORD0;
    float3 normalWS : TEXCOORD1;
    float2 heightData : TEXCOORD2;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    float4 worldPos = mul(float4(input.position, 1.0), gModel);
    o.position = mul(worldPos, gViewProj);
    o.worldPos = worldPos.xyz;
    o.normalWS = normalize(mul(float4(input.normal, 0.0), gModel).xyz);
    o.heightData = input.uv;
    return o;
}

float InterleavedGradientNoise(float2 pixel, float frame)
{
    float3 magic = float3(0.06711056, 0.00583715, 52.9829189);
    return frac(magic.z * frac(dot(pixel + frame, magic.xy)));
}

float3 TerrainColor(float heightMeters)
{
    if (heightMeters <= 0.5)
    {
        return float3(0.05, 0.18, 0.40);
    }

    float h01 = saturate(heightMeters / max(gColorAndFlags.y, 1.0));
    float3 low = float3(0.17, 0.43, 0.16);
    float3 mid = float3(0.38, 0.32, 0.20);
    float3 high = float3(0.82, 0.82, 0.84);
    return (h01 < 0.55) ? lerp(low, mid, h01 / 0.55) : lerp(mid, high, (h01 - 0.55) / 0.45);
}

float LatDegToMercatorY(float latDeg)
{
    const float clampedLat = clamp(latDeg, -85.05112878, 85.05112878);
    const float latRad = radians(clampedLat);
    return log(tan(latRad) + rcp(cos(latRad)));
}

float2 MapLonLatToRingUv(float lonDeg, float latDeg, float4 boundsLonLat, out bool inside)
{
    inside = false;
    const float lonSpan = boundsLonLat.y - boundsLonLat.x;
    const float mercSouth = LatDegToMercatorY(boundsLonLat.z);
    const float mercNorth = LatDegToMercatorY(boundsLonLat.w);
    const float mercSpan = mercNorth - mercSouth;
    if (abs(lonSpan) < 1e-5 || abs(mercSpan) < 1e-6)
    {
        return float2(0.0, 0.0);
    }

    float lonAdj = lonDeg;
    if (lonAdj < boundsLonLat.x)
    {
        lonAdj += 360.0;
    }
    if (lonAdj > boundsLonLat.y)
    {
        lonAdj -= 360.0;
    }

    const float u = (lonAdj - boundsLonLat.x) / lonSpan;
    const float mercLat = LatDegToMercatorY(latDeg);
    const float v = (mercNorth - mercLat) / mercSpan;
    inside = (u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0);
    return float2(u, v);
}

uint WorldHashTileKey(uint z, uint x, uint y)
{
    uint h = 2166136261u;
    h = (h ^ z) * 16777619u;
    h = (h ^ x) * 16777619u;
    h = (h ^ y) * 16777619u;
    return h;
}

uint PackWorldKey0(uint z, uint x)
{
    return ((z & 63u) << 26u) | (x & 0x03FFFFFFu);
}

uint PackWorldKey1(uint y)
{
    return y & 0x03FFFFFFu;
}

uint WrapTileXSigned(int x, uint n)
{
    int r = x % int(n);
    if (r < 0)
    {
        r += int(n);
    }
    return uint(r);
}

bool LookupWorldAtlasPage(uint z, uint x, uint y, out uint atlasPageIndex)
{
    atlasPageIndex = 0u;
    const uint tableW = max(uint(gTuning13.z), 1u);
    const uint tableH = max(uint(gTuning13.w), 1u);
    const uint slotCount = tableW * tableH;
    if (slotCount == 0u)
    {
        return false;
    }

    const uint key0 = PackWorldKey0(z, x);
    const uint key1 = PackWorldKey1(y);
    const uint start = WorldHashTileKey(z, x, y) % slotCount;
    const uint maxProbe = min(max(uint(gTuning14.w), 1u), 16u);
    [loop]
    for (uint p = 0u; p < maxProbe; ++p)
    {
        const uint slot = (start + p) % slotCount;
        const uint2 coord = uint2(slot % tableW, slot / tableW);
        const uint value = gWorldSatellitePageTable.Load(int3(coord, 0));
        if ((value & 0x80000000u) == 0u)
        {
            // Zero means never-used slot (end of chain). Tombstones have 0x40000000 set.
            if (value == 0u)
            {
                return false;
            }
            continue;
        }

        const uint2 packedKey = gWorldSatellitePageKeys.Load(int3(coord, 0));
        if (packedKey.x == key0 && packedKey.y == key1 && ((value & 0x80000000u) != 0u))
        {
            atlasPageIndex = value & 0x00FFFFFFu;
            return true;
        }
    }
    return false;
}

float ComputeWorldAtlasLod(float2 tileCoord)
{
    // Derive mip level from continuous tile coordinates instead of frac(localUv),
    // which is discontinuous at every tile edge and produces seam-only mip spikes.
    const float2 gradXTexels = ddx(tileCoord) * 256.0;
    const float2 gradYTexels = ddy(tileCoord) * 256.0;
    const float rho2 = max(dot(gradXTexels, gradXTexels), dot(gradYTexels, gradYTexels));
    return clamp(0.5 * log2(max(rho2, 1.0e-8)) - 0.15, 0.0, 8.0);
}

bool SampleWorldAtlasPageLevel(int z, int tileXi, int tileYi, float2 localUv, float lod, out float4 sampleOut)
{
    sampleOut = 0.0;
    if (z < 0 || z > 22)
    {
        return false;
    }

    const uint n = (1u << uint(z));
    const uint x = WrapTileXSigned(tileXi, n);
    const uint y = uint(clamp(tileYi, 0, int(n) - 1));

    uint atlasPageIndex = 0u;
    if (!LookupWorldAtlasPage(uint(z), x, y, atlasPageIndex))
    {
        return false;
    }

    const uint atlasPagesX = max(uint(gTuning13.x), 1u);
    const uint atlasPagesY = max(uint(gTuning13.y), 1u);
    const uint atlasPageCount = atlasPagesX * atlasPagesY;
    if (atlasPageIndex >= atlasPageCount)
    {
        return false;
    }

    const uint pageX = atlasPageIndex % atlasPagesX;
    const uint pageY = atlasPageIndex / atlasPagesX;

    const float2 atlasPageScale = rcp(float2(atlasPagesX, atlasPagesY));
    // Expand the border guard with mip level so filtering never pulls from a neighboring atlas page.
    const float mipGuardPixels = 0.5 * exp2(lod) + 1.0;
    const float guard = saturate(mipGuardPixels / 256.0);
    const float2 guardedUv = clamp(localUv, float2(guard, guard), float2(1.0 - guard, 1.0 - guard));
    const float2 atlasUv = (float2(pageX, pageY) + guardedUv) * atlasPageScale;
    sampleOut = gWorldSatelliteAtlas.SampleLevel(gClampSampler, atlasUv, lod);
    return sampleOut.a > 0.001;
}

bool SampleWorldAtlasTile(int z, float2 tileCoord, int tileXi, int tileYi, float2 localUv, out float4 sampleOut)
{
    sampleOut = 0.0;
    if (z < 0 || z > 22)
    {
        return false;
    }

    const float lod = ComputeWorldAtlasLod(tileCoord);
    return SampleWorldAtlasPageLevel(z, tileXi, tileYi, localUv, lod, sampleOut);
}

bool SampleWorldLonLatSeamAware(float lonDeg, float latDeg, int z, out float4 sampleOut)
{
    sampleOut = 0.0;
    if (z < 0 || z > 22)
    {
        return false;
    }

    const float n = exp2(float(z));
    const float x = (lonDeg + 180.0) / 360.0 * n;
    const float latRad = radians(clamp(latDeg, -85.05112878, 85.05112878));
    const float y = (1.0 - log(tan(latRad) + rcp(cos(latRad))) / kPi) * 0.5 * n;

    const int tileXi = int(floor(x));
    const int tileYi = int(floor(y));
    const float2 tileCoord = float2(x, y);
    const float2 localUv = float2(frac(x), frac(y));

    return SampleWorldAtlasTile(z, tileCoord, tileXi, tileYi, localUv, sampleOut);
}

bool SampleWorldHierarchical(
    float lonDeg,
    float latDeg,
    int preferredZoom,
    int fallbackZoomA,
    int fallbackZoomB,
    out float4 sampleOut)
{
    sampleOut = 0.0;
    const int z0 = clamp(preferredZoom, 0, 22);
    const int z1 = clamp(preferredZoom - 1, 0, 22);
    const int z2 = clamp(fallbackZoomA, 0, 22);
    const int z3 = clamp(fallbackZoomB, 0, 22);

    float4 s = 0.0;
    if (SampleWorldLonLatSeamAware(lonDeg, latDeg, z0, s)) { sampleOut = s; return true; }
    if (SampleWorldLonLatSeamAware(lonDeg, latDeg, z1, s)) { sampleOut = s; return true; }
    if (SampleWorldLonLatSeamAware(lonDeg, latDeg, z2, s)) { sampleOut = s; return true; }
    if (SampleWorldLonLatSeamAware(lonDeg, latDeg, z3, s)) { sampleOut = s; return true; }
    return false;
}

float4 PSMain(VSOutput input) : SV_Target
{
    float clampedHeight = input.heightData.x;

    float3 N = normalize(input.normalWS);
    float3 V = normalize(-input.worldPos);
    float3 L = normalize(gSunDirIntensity.xyz);

    float ndl = saturate(dot(N, L));
    float hemi = saturate(dot(N, normalize(gCameraUpAndTime.xyz)) * 0.5 + 0.5);

    float ambient = lerp(0.09, 0.25, hemi);
    float diffuse = 0.80 * ndl;
    float3 H = normalize(V + L);
    float spec = pow(saturate(dot(N, H)), 48.0) * gTuning1.z;
    const float distMeters = length(input.worldPos);
    const float nearLodRadius = max(gColorAndFlags.x, 1000.0);
    const float transitionWidth = max(gColorAndFlags.z, 500.0);
    const float boundary0 = nearLodRadius;
    const float boundary1 = nearLodRadius * max(gTuning0.x, 1.1);
    const float boundary2 = boundary1 * max(gTuning0.y, 1.1);

    float3 baseColor = TerrainColor(clampedHeight);
    if (gTuning2.x > 0.5)
    {
        // Map world position to lat/lon using the vector from Earth center.
        // This is translation-invariant under the floating-origin anchor: (p - anchor) - (center - anchor) = p - center.
        float3 dir = normalize(input.worldPos - PlanetCenter());
        float lat = asin(clamp(dir.y, -1.0, 1.0));
        float lon = atan2(dir.z, dir.x);
        const float latDeg = lat * (180.0 / kPi);
        const float lonDeg = lon * (180.0 / kPi);

        // Base fallback from local GeoTIFF bounds.
        const float lonWest = gTuning3.x;
        const float lonEast = gTuning3.y;
        const float latSouth = gTuning3.z;
        const float latNorth = gTuning3.w;
        const float lonSpan = max(1e-6, lonEast - lonWest);
        const float latSpan = max(1e-6, latNorth - latSouth);

        float u = (lonDeg - lonWest) / lonSpan;
        float v = (latNorth - latDeg) / latSpan;

        const bool wrapLon = (gTuning2.z > 0.5);
        float3 satelliteColor = baseColor;
        bool haveFallback = false;
        if (wrapLon)
        {
            u = frac(u);
            v = saturate(v);
            satelliteColor = gEarthAlbedo.Sample(gWrapSampler, float2(u, v)).rgb;
            haveFallback = true;
        }
        else
        {
            // For regional datasets, avoid smearing edges by skipping samples outside the covered bounds.
            if (u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0)
            {
                satelliteColor = gEarthAlbedo.Sample(gClampSampler, float2(u, v)).rgb;
                haveFallback = true;
            }
        }

        const float nearWeight = 1.0 - saturate(smoothstep(boundary0 - transitionWidth, boundary1 + transitionWidth, distMeters));
        const float farWeight = saturate(smoothstep(boundary1 - transitionWidth, boundary2 + transitionWidth, distMeters));
        const float midWeight = saturate(1.0 - nearWeight - farWeight);

        const bool worldLockedEnabled = (gTuning12.y > 0.5);
        if (worldLockedEnabled)
        {
            int nearZoom = int(gTuning14.x + 0.5);
            int midZoom = int(gTuning14.y + 0.5);
            int farZoom = int(gTuning14.z + 0.5);

            // Fallback if CPU profile is invalid for any reason.
            if (nearZoom <= 0 || midZoom <= 0 || farZoom <= 0)
            {
                const float camAltMeters = max(0.0, length(CameraPosition() - PlanetCenter()) - GroundRadiusMeters());
                if (camAltMeters < 500.0)      { nearZoom = 16; midZoom = 15; farZoom = 13; }
                else if (camAltMeters < 2000.0) { nearZoom = 15; midZoom = 14; farZoom = 12; }
                else if (camAltMeters < 6000.0) { nearZoom = 14; midZoom = 13; farZoom = 11; }
                else                            { nearZoom = 13; midZoom = 12; farZoom = 10; }
            }

            float3 worldAccum = 0.0;
            float worldW = 0.0;

            if (nearWeight > 1e-4)
            {
                float4 s;
                if (SampleWorldHierarchical(lonDeg, latDeg, nearZoom, midZoom, farZoom, s))
                {
                    const float w = nearWeight * saturate(max(s.a, 0.001));
                    worldAccum += s.rgb * w;
                    worldW += w;
                }
            }
            if (midWeight > 1e-4)
            {
                float4 s;
                if (SampleWorldHierarchical(lonDeg, latDeg, midZoom, farZoom, nearZoom - 2, s))
                {
                    const float w = midWeight * saturate(max(s.a, 0.001));
                    worldAccum += s.rgb * w;
                    worldW += w;
                }
            }
            if (farWeight > 1e-4)
            {
                float4 s;
                if (SampleWorldHierarchical(lonDeg, latDeg, farZoom, farZoom - 1, farZoom - 2, s))
                {
                    const float w = farWeight * saturate(max(s.a, 0.001));
                    worldAccum += s.rgb * w;
                    worldW += w;
                }
            }

            if (worldW > 1e-4)
            {
                const float3 streamed = worldAccum / worldW;
                // Keep coverage stable once we have any streamed sample this frame.
                const float blend = saturate(gTuning12.z);
                satelliteColor = haveFallback ? lerp(satelliteColor, streamed, blend) : streamed;
                haveFallback = true;
            }
        }
        else if (gTuning2.w > 0.5)
        {
            float3 streamAccum = 0.0;
            float streamW = 0.0;
            float3 prevAccum = 0.0;
            float prevW = 0.0;

            if (gTuning7.x > 0.5 && nearWeight > 1e-4)
            {
                bool inNear = false;
                const float2 uvNear = MapLonLatToRingUv(lonDeg, latDeg, gTuning4, inNear);
                if (inNear)
                {
                    const float4 s = gSatelliteNear.Sample(gClampSampler, uvNear);
                    streamAccum += s.rgb * (nearWeight * s.a);
                    streamW += nearWeight * s.a;
                }
            }
            if (gTuning11.x > 0.5 && nearWeight > 1e-4)
            {
                bool inNear = false;
                const float2 uvNear = MapLonLatToRingUv(lonDeg, latDeg, gTuning8, inNear);
                if (inNear)
                {
                    const float4 s = gSatellitePrevNear.Sample(gClampSampler, uvNear);
                    prevAccum += s.rgb * (nearWeight * s.a);
                    prevW += nearWeight * s.a;
                }
            }
            if (gTuning7.y > 0.5 && midWeight > 1e-4)
            {
                bool inMid = false;
                const float2 uvMid = MapLonLatToRingUv(lonDeg, latDeg, gTuning5, inMid);
                if (inMid)
                {
                    const float4 s = gSatelliteMid.Sample(gClampSampler, uvMid);
                    streamAccum += s.rgb * (midWeight * s.a);
                    streamW += midWeight * s.a;
                }
            }
            if (gTuning11.y > 0.5 && midWeight > 1e-4)
            {
                bool inMid = false;
                const float2 uvMid = MapLonLatToRingUv(lonDeg, latDeg, gTuning9, inMid);
                if (inMid)
                {
                    const float4 s = gSatellitePrevMid.Sample(gClampSampler, uvMid);
                    prevAccum += s.rgb * (midWeight * s.a);
                    prevW += midWeight * s.a;
                }
            }
            if (gTuning7.z > 0.5 && farWeight > 1e-4)
            {
                bool inFar = false;
                const float2 uvFar = MapLonLatToRingUv(lonDeg, latDeg, gTuning6, inFar);
                if (inFar)
                {
                    const float4 s = gSatelliteFar.Sample(gClampSampler, uvFar);
                    streamAccum += s.rgb * (farWeight * s.a);
                    streamW += farWeight * s.a;
                }
            }
            if (gTuning11.z > 0.5 && farWeight > 1e-4)
            {
                bool inFar = false;
                const float2 uvFar = MapLonLatToRingUv(lonDeg, latDeg, gTuning10, inFar);
                if (inFar)
                {
                    const float4 s = gSatellitePrevFar.Sample(gClampSampler, uvFar);
                    prevAccum += s.rgb * (farWeight * s.a);
                    prevW += farWeight * s.a;
                }
            }

            const bool hasCurrent = streamW > 1e-4;
            const bool hasPrev = prevW > 1e-4;
            if (hasCurrent || hasPrev)
            {
                const float3 currentColor = hasCurrent ? (streamAccum / streamW) : (hasPrev ? (prevAccum / prevW) : satelliteColor);
                const float3 prevColor = hasPrev ? (prevAccum / prevW) : currentColor;
                const float transition = saturate(gTuning11.w);
                const float3 streamed = lerp(prevColor, currentColor, transition);
                const float streamAlpha = saturate(max(streamW, prevW));
                satelliteColor = haveFallback ? lerp(satelliteColor, streamed, streamAlpha) : streamed;
                haveFallback = true;
            }
        }

        if (haveFallback)
        {
            baseColor = lerp(baseColor, satelliteColor, saturate(gTuning2.y));
        }
    }
    const float slopeTerm = saturate(1.0 - abs(dot(N, normalize(gCameraUpAndTime.xyz))));
    const float slopeBoost = 1.0 + slopeTerm * gTuning1.y;
    baseColor *= slopeBoost;
    float3 color = baseColor * (ambient + diffuse) + spec.xxx;
    color = pow(max(color, 1e-4), 1.0 / max(gTuning1.x, 0.05));

    const float frameId = floor(gCameraUpAndTime.w * 10.0);
    const float dither = InterleavedGradientNoise(floor(input.position.xy), frameId) - 0.5;
    const float blend0 = saturate(smoothstep(boundary0 - transitionWidth, boundary0 + transitionWidth, distMeters) + dither * 0.18);
    const float blend1 = saturate(smoothstep(boundary1 - transitionWidth, boundary1 + transitionWidth, distMeters) + dither * 0.14);
    const float blend2 = saturate(smoothstep(boundary2 - transitionWidth, boundary2 + transitionWidth, distMeters) + dither * 0.10);
    const float seamMask = max(
        max(1.0 - saturate(abs(distMeters - boundary0) / transitionWidth),
            1.0 - saturate(abs(distMeters - boundary1) / transitionWidth)),
        1.0 - saturate(abs(distMeters - boundary2) / transitionWidth));

    // Dithered LOD transition softening to hide ring swaps.
    const float lodBlend = saturate((blend0 + blend1 + blend2) / 3.0);
    const float seamBlend = saturate(seamMask * gTuning1.w + lodBlend * 0.12);
    color *= lerp(1.0, 0.985, seamBlend * 0.35);

    // Optional artistic haze (non-physical). Keep at 0 for realistic mode.
    if (gTuning0.z > 1e-4)
    {
        const float3 skyDir = normalize(input.worldPos);
        const float2 skyUv = DirectionToSkyViewUv(skyDir, normalize(gCameraUpAndTime.xyz), L);
        const float3 skyColor = gSkyViewLut.SampleLevel(gClampSampler, skyUv, 0).rgb;
        const float cameraAltitude = max(0.0, length(CameraPosition() - PlanetCenter()) - GroundRadiusMeters());
        const float lowAltitudeFactor = saturate(1.0 - cameraAltitude / max(gTuning0.w, 1000.0));
        const float farFieldFactor = saturate(distMeters / max(gColorAndFlags.w, 1.0));
        const float extraHaze = lowAltitudeFactor * farFieldFactor * farFieldFactor;
        color = lerp(color, skyColor, extraHaze * gTuning0.z);
    }

    color = ApplyAerialPerspectiveToColor(gAerialPerspectiveLut, gClampSampler, input.position.xy, length(input.worldPos), color);
    color = 1.0 - exp(-color * max(gAtmosphereFlags.z, 0.01));
    return float4(saturate(color), saturate(gTuning12.x));
}
