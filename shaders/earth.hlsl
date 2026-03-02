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
        float mask = gLandmask.Sample(gSampler, input.uv).r;
        baseColor = (mask > 0.5) ? land : water;
    }
    else
    {
        float band = saturate((N.y + 0.15) * 1.2);
        baseColor = lerp(water, float3(0.20, 0.45, 0.25), 0.35 * band);
    }

    float ndl = saturate(dot(N, L));
    float hemi = saturate(N.y * 0.5 + 0.5);

    // Subtle water glancing-angle brightening.
    float fresnel = pow(1.0 - saturate(dot(N, V)), 5.0);
    float waterBoost = (1.0 - gColorAndFlags.a) * 0.25 * fresnel;

    float ambient = lerp(0.12, 0.30, hemi);
    float diffuse = 0.78 * ndl;
    float3 color = baseColor * (ambient + diffuse + waterBoost);

    // Atmospheric rim and horizon haze for daytime Earth look.
    float rim = pow(1.0 - saturate(dot(N, V)), 3.0);
    float sunScatter = pow(saturate(dot(V, L)), 4.0);
    float3 skyTint = gSkybox.Sample(gSampler, N).rgb;
    float3 atmoColor = lerp(float3(0.35, 0.56, 0.92), skyTint, 0.45) * (0.30 + 0.70 * sunScatter);
    color += atmoColor * rim * gAtmosphereParams.x;

    float horizon = pow(1.0 - abs(V.y), 2.0);
    color += float3(0.10, 0.15, 0.22) * horizon * gAtmosphereParams.y;

    return float4(saturate(color), 1.0);
}
