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
    float spec = pow(saturate(dot(N, H)), 48.0) * 0.14;

    float3 baseColor = TerrainColor(clampedHeight);
    float3 color = baseColor * (ambient + diffuse) + spec.xxx;

    color = ApplyAerialPerspectiveToColor(gAerialPerspectiveLut, gClampSampler, input.position.xy, length(input.worldPos), color);
    color = 1.0 - exp(-color * max(gAtmosphereFlags.z, 0.01));
    return float4(saturate(color), 1.0);
}
