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
};

struct VSInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float2 uv : TEXCOORD0;
};

struct VSOutput
{
    float4 position : SV_Position;
    float3 dir : TEXCOORD0;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    float4 worldPos = mul(float4(input.position, 1.0), gModel);
    o.position = mul(worldPos, gViewProj);
    o.dir = worldPos.xyz;
    return o;
}

float SunDisk(float cosAngle)
{
    float sunRadius = max(gOzoneHalfWidthAtmosphereHeightSunRadius.z, 1e-4);
    float cosInner = cos(sunRadius);
    float cosOuter = cos(sunRadius * 1.35);
    return smoothstep(cosOuter, cosInner, cosAngle);
}

float4 PSMain(VSOutput input) : SV_Target
{
    float3 dir = normalize(input.dir);
    float3 sunDir = normalize(gSunDirIntensity.xyz);
    float3 color = 0.0;

    if (gViewportAndAerialDepth.w > 0.5)
    {
        float2 uv = DirectionToSkyViewUv(dir, normalize(gCameraUpAndTime.xyz), sunDir);
        color = gSkyViewLut.SampleLevel(gClampSampler, uv, 0).rgb;

        // Composite sun disk after applying the Sky-View LUT (paper recommendation).
        float cosAngle = dot(dir, sunDir);
        float disk = SunDisk(cosAngle);
        float halo = pow(saturate(cosAngle), 96.0);
        float3 sunColor = float3(1.0, 0.95, 0.84) * (0.35 * gSunDirIntensity.w);
        color += sunColor * (disk * 6.0 + halo * 0.12);
    }
    else
    {
        color = gSkybox.Sample(gWrapSampler, dir).rgb;
    }

    color = 1.0 - exp(-color * max(gAtmosphereFlags.z, 0.01));
    return float4(saturate(color), 1.0);
}
