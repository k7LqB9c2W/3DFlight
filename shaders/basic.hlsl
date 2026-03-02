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
    float3 lightDir = normalize(float3(0.4, 0.7, 0.2));
    float ndl = saturate(dot(normalize(input.normalWS), lightDir));
    float lighting = 0.25 + 0.75 * ndl;
    return float4(gColorAndFlags.rgb * lighting, 1.0);
}
