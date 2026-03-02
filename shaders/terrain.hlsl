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
    const float seaLevel = 0.0;
    if (heightMeters <= seaLevel + 0.5)
    {
        return float3(0.07, 0.23, 0.53);
    }

    float h01 = saturate(heightMeters / max(gColorAndFlags.y, 1.0));
    float3 low = float3(0.17, 0.43, 0.16);
    float3 mid = float3(0.38, 0.32, 0.20);
    float3 high = float3(0.82, 0.82, 0.84);

    float3 c = (h01 < 0.55)
        ? lerp(low, mid, h01 / 0.55)
        : lerp(mid, high, (h01 - 0.55) / 0.45);
    return c;
}

float4 PSMain(VSOutput input) : SV_Target
{
    float clampedHeight = input.heightData.x;

    float3 N = normalize(input.normalWS);
    float3 V = normalize(-input.worldPos);
    float3 L = normalize(gSunDirIntensity.xyz);

    float ndl = saturate(dot(N, L));
    float hemi = saturate(N.y * 0.5 + 0.5);

    float ambient = lerp(0.10, 0.30, hemi);
    float diffuse = 0.80 * ndl;

    float3 H = normalize(V + L);
    float spec = pow(saturate(dot(N, H)), 48.0) * 0.16;

    float3 baseColor = TerrainColor(clampedHeight);
    float3 color = baseColor * (ambient + diffuse) + spec.xxx;

    // Daytime aerial haze.
    float horizon = pow(1.0 - abs(V.y), 1.8);
    float3 sky = gSkybox.Sample(gSampler, float3(0.0, 1.0, 0.0)).rgb;
    color = lerp(color, sky, 0.18 * horizon);

    return float4(saturate(color), 1.0);
}
