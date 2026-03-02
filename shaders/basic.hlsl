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
    float3 N = normalize(input.normalWS);
    float3 V = normalize(-input.worldPos);
    float3 L = normalize(gSunDirIntensity.xyz);

    float ndl = saturate(dot(N, L));
    float hemi = saturate(N.y * 0.5 + 0.5);

    float3 H = normalize(L + V);
    float spec = pow(saturate(dot(N, H)), 64.0) * 0.25;
    float rim = pow(1.0 - saturate(dot(N, V)), 4.0) * 0.18;

    float ambient = lerp(0.10, 0.28, hemi);
    float diffuse = 0.85 * ndl;

    float3 baseColor = gColorAndFlags.rgb;
    float3 lit = baseColor * (ambient + diffuse) + spec.xxx + rim * float3(0.60, 0.70, 0.85);

    return float4(saturate(lit), 1.0);
}
