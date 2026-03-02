Texture2D gLandmask : register(t0);
TextureCube gSkybox : register(t1);
SamplerState gSampler : register(s0);

cbuffer SceneCB : register(b0)
{
    float4x4 gViewProj;
    float4 gEarthCenterRadius;
    float4 gSunDirIntensity;
    float4 gAtmosphereParams;
};

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

float4 PSMain(VSOutput input) : SV_Target
{
    float3 dir = normalize(input.dir);
    float3 color = gSkybox.Sample(gSampler, dir).rgb;

    // Light daytime enhancement around sun direction.
    float sunAmount = pow(saturate(dot(dir, normalize(gSunDirIntensity.xyz))), 180.0);
    color += float3(1.0, 0.93, 0.80) * sunAmount * 0.35;

    return float4(saturate(color), 1.0);
}
