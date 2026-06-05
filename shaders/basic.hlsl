#include "atmosphere_common.hlsli"

Texture2D<float> gLandmask : register(t0);
TextureCube<float4> gSkybox : register(t1);
Texture2D<float4> gTransmittanceLut : register(t2);
Texture2D<float4> gSkyViewLut : register(t3);
Texture2D<float4> gMultipleScatteringLut : register(t4);
Texture3D<float4> gAerialPerspectiveLut : register(t5);
Texture2D<float4> gEarthAlbedo : register(t6);
Texture2D<float4> gModelAlbedo : register(t30);

SamplerState gWrapSampler : register(s0);
SamplerState gClampSampler : register(s1);

cbuffer ObjectCB : register(b1)
{
    float4x4 gModel;
    float4 gColorAndFlags;
    float4 gTuning0; // x alphaCutoff, y emissive, z transparent, w unused
};

struct VSInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float2 uv : TEXCOORD0;
    float4 color : COLOR0;
};

struct VSOutput
{
    float4 position : SV_Position;
    float3 worldPos : TEXCOORD0;
    float3 normalWS : TEXCOORD1;
    float2 uv : TEXCOORD2;
    float4 color : TEXCOORD3;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    float4 worldPos = mul(float4(input.position, 1.0), gModel);
    o.position = mul(worldPos, gViewProj);
    o.worldPos = worldPos.xyz;
    o.normalWS = normalize(mul(float4(input.normal, 0.0), gModel).xyz);
    o.uv = input.uv;
    o.color = input.color;
    return o;
}

float4 PSMain(VSOutput input) : SV_Target
{
    float3 N = normalize(input.normalWS);
    float3 V = normalize(-input.worldPos);
    float3 L = normalize(gSunDirIntensity.xyz);

    float ndl = saturate(dot(N, L));
    float hemi = saturate(dot(N, normalize(gCameraUpAndTime.xyz)) * 0.5 + 0.5);

    float3 H = normalize(L + V);
    float spec = pow(saturate(dot(N, H)), 64.0) * 0.25;
    float rim = pow(1.0 - saturate(dot(N, V)), 4.0) * 0.18;

    float ambient = lerp(0.08, 0.25, hemi);
    float diffuse = 0.85 * ndl;

    float alpha = input.color.a;
    float3 baseColor = gColorAndFlags.rgb * input.color.rgb;
    if (gColorAndFlags.w > 0.5)
    {
        float4 albedo = gModelAlbedo.Sample(gWrapSampler, input.uv);
        baseColor *= albedo.rgb;
        float sampledAlpha = albedo.a;
        if (gTuning0.y > 0.5 && sampledAlpha > 0.999)
        {
            sampledAlpha = saturate(dot(albedo.rgb, float3(0.299, 0.587, 0.114)));
        }
        alpha *= sampledAlpha;
    }
    clip(alpha - max(gTuning0.x, 0.0));

    float3 lit = baseColor * (ambient + diffuse) + spec.xxx + rim * float3(0.52, 0.62, 0.75);
    if (gTuning0.y > 0.5)
    {
        lit = max(lit, baseColor * 2.5);
    }

    lit = ApplyAerialPerspectiveToColor(gAerialPerspectiveLut, gClampSampler, input.position.xy, length(input.worldPos), lit);
    lit = 1.0 - exp(-lit * max(gAtmosphereFlags.z, 0.01));
    return float4(saturate(lit), saturate(alpha));
}
