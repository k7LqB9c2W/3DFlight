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
    float3 worldPos : TEXCOORD0;
    float3 normalWS : TEXCOORD1;
    float2 uv : TEXCOORD2;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    float4 worldPos = mul(float4(input.position, 1.0), gModel);
    o.position = mul(worldPos, gViewProj);
    o.worldPos = worldPos.xyz;
    o.normalWS = normalize(mul(float4(input.normal, 0.0), gModel).xyz);
    o.uv = input.uv;
    return o;
}

float4 PSMain(VSOutput input) : SV_Target
{
    const float3 water = float3(0.07, 0.23, 0.53);
    const float3 land = float3(0.16, 0.55, 0.24);

    float3 N = normalize(input.normalWS);
    float3 V = normalize(-input.worldPos);
    float3 L = normalize(gSunDirIntensity.xyz);

    float3 baseColor = water;
    if (gColorAndFlags.a > 0.5)
    {
        float mask = gLandmask.Sample(gWrapSampler, input.uv).r;
        baseColor = (mask > 0.5) ? land : water;
    }
    else
    {
        float band = saturate((dot(N, normalize(gCameraUpAndTime.xyz)) + 0.15) * 1.2);
        baseColor = lerp(water, float3(0.20, 0.45, 0.25), 0.35 * band);
    }

    float ndl = saturate(dot(N, L));
    float hemi = saturate(dot(N, normalize(gCameraUpAndTime.xyz)) * 0.5 + 0.5);

    float fresnel = pow(1.0 - saturate(dot(N, V)), 5.0);
    float waterBoost = (1.0 - gColorAndFlags.a) * 0.20 * fresnel;

    float ambient = lerp(0.10, 0.27, hemi);
    float diffuse = 0.78 * ndl;
    float3 color = baseColor * (ambient + diffuse + waterBoost);

    float rim = pow(1.0 - saturate(dot(N, V)), 3.0);
    color += float3(0.30, 0.43, 0.68) * rim * 0.30;

    color = ApplyAerialPerspectiveToColor(gAerialPerspectiveLut, gClampSampler, input.position.xy, length(input.worldPos), color);
    color = 1.0 - exp(-color * max(gAtmosphereFlags.z, 0.01));
    return float4(saturate(color), 1.0);
}
