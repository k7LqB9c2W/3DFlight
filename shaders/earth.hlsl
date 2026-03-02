Texture2D gLandmask : register(t0);
SamplerState gSampler : register(s0);

cbuffer SceneCB : register(b0)
{
    float4x4 gViewProj;
    float4 gEarthCenterRadius;
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
    float3 normalWS : NORMAL;
    float2 uv : TEXCOORD0;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    float4 worldPos = mul(float4(input.position, 1.0), gModel);
    o.position = mul(worldPos, gViewProj);
    o.normalWS = normalize(mul(float4(input.normal, 0.0), gModel).xyz);
    o.uv = input.uv;
    return o;
}

float4 PSMain(VSOutput input) : SV_Target
{
    const float3 water = float3(0.07, 0.23, 0.53);
    const float3 land = float3(0.16, 0.55, 0.24);

    float3 baseColor = water;
    if (gColorAndFlags.a > 0.5)
    {
        float mask = gLandmask.Sample(gSampler, input.uv).r;
        baseColor = (mask > 0.5) ? land : water;
    }
    else
    {
        float band = saturate((input.normalWS.y + 0.15) * 1.2);
        baseColor = lerp(water, float3(0.20, 0.45, 0.25), 0.35 * band);
    }

    float3 lightDir = normalize(float3(0.5, 0.8, -0.1));
    float ndl = saturate(dot(normalize(input.normalWS), lightDir));
    float lighting = 0.30 + 0.70 * ndl;

    return float4(baseColor * lighting, 1.0);
}
