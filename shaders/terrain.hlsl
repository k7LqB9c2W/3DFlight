#include "atmosphere_common.hlsli"

Texture2D<float> gLandmask : register(t0);
TextureCube<float4> gSkybox : register(t1);
Texture2D<float4> gTransmittanceLut : register(t2);
Texture2D<float4> gSkyViewLut : register(t3);
Texture2D<float4> gMultipleScatteringLut : register(t4);
Texture3D<float4> gAerialPerspectiveLut : register(t5);

SamplerState gWrapSampler : register(s0);
SamplerState gClampSampler : register(s1);

cbuffer ObjectCB : register(b1)
{
    float4x4 gModel;
    float4 gColorAndFlags;
    float4 gTuning0; // x midRingMul, y farRingMul, z hazeStrength, w hazeAltitudeRangeMeters
    float4 gTuning1; // x contrast, y slopeBoost, z specularStrength, w lodSeamBlendStrength
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

    float3 baseColor = TerrainColor(clampedHeight);
    const float slopeTerm = saturate(1.0 - abs(dot(N, normalize(gCameraUpAndTime.xyz))));
    const float slopeBoost = 1.0 + slopeTerm * gTuning1.y;
    baseColor *= slopeBoost;
    float3 color = baseColor * (ambient + diffuse) + spec.xxx;
    color = pow(max(color, 1e-4), 1.0 / max(gTuning1.x, 0.05));

    const float distMeters = length(input.worldPos);
    const float nearLodRadius = max(gColorAndFlags.x, 1000.0);
    const float transitionWidth = max(gColorAndFlags.z, 500.0);
    const float boundary0 = nearLodRadius;
    const float boundary1 = nearLodRadius * max(gTuning0.x, 1.1);
    const float boundary2 = boundary1 * max(gTuning0.y, 1.1);

    const float frameId = floor(gCameraUpAndTime.w * 10.0);
    const float dither = InterleavedGradientNoise(floor(input.position.xy), frameId) - 0.5;
    const float blend0 = saturate(smoothstep(boundary0 - transitionWidth, boundary0 + transitionWidth, distMeters) + dither * 0.18);
    const float blend1 = saturate(smoothstep(boundary1 - transitionWidth, boundary1 + transitionWidth, distMeters) + dither * 0.14);
    const float blend2 = saturate(smoothstep(boundary2 - transitionWidth, boundary2 + transitionWidth, distMeters) + dither * 0.10);
    const float seamMask = max(
        max(1.0 - saturate(abs(distMeters - boundary0) / transitionWidth),
            1.0 - saturate(abs(distMeters - boundary1) / transitionWidth)),
        1.0 - saturate(abs(distMeters - boundary2) / transitionWidth));

    const float3 skyDir = normalize(input.worldPos);
    const float2 skyUv = DirectionToSkyViewUv(skyDir, normalize(gCameraUpAndTime.xyz), L);
    const float3 skyColor = gSkyViewLut.SampleLevel(gClampSampler, skyUv, 0).rgb;

    // Dithered LOD transition softening to hide ring swaps.
    const float lodBlend = saturate((blend0 + blend1 + blend2) / 3.0);
    const float seamBlend = saturate(seamMask * gTuning1.w + lodBlend * 0.12);
    color = lerp(color, color * 0.93 + skyColor * 0.07, seamBlend);

    // Altitude-dependent distance haze (stronger close to sea level).
    const float cameraAltitude = max(0.0, length(CameraPosition() - PlanetCenter()) - GroundRadiusMeters());
    const float lowAltitudeFactor = saturate(1.0 - cameraAltitude / max(gTuning0.w, 1000.0));
    const float farFieldFactor = saturate(distMeters / max(gColorAndFlags.w, 1.0));
    const float extraHaze = lowAltitudeFactor * farFieldFactor * farFieldFactor;
    color = lerp(color, skyColor, extraHaze * gTuning0.z);

    color = ApplyAerialPerspectiveToColor(gAerialPerspectiveLut, gClampSampler, input.position.xy, length(input.worldPos), color);
    color = 1.0 - exp(-color * max(gAtmosphereFlags.z, 0.01));
    return float4(saturate(color), 1.0);
}
