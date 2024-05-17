#ifndef VERTEX_H
#define VERTEX_H

#include "UnityCG.cginc"

struct v_in
{
    float4 vertex : POSITION;
    float2 uv : TEXCOORD0;
};

struct v_out
{
    float2 pixel : TEXCOORD0;
    UNITY_FOG_COORDS(1)
    float4 vertex : SV_POSITION;
};

float4 mainImage(in float2 pixel);

sampler2D _MainTex;
float4 _MainTex_ST;

v_out vert(v_in v)
{
    v_out o;
    o.vertex = UnityObjectToClipPos(v.vertex);
    o.pixel = ComputeScreenPos(o.vertex) * _ScreenParams.xy;
    UNITY_TRANSFER_FOG(o, o.vertex);
    return o;
}

fixed4 frag(v_out i) : SV_Target
{
    // sample the texture
    fixed4 col = mainImage(i.pixel);
    // apply fog
    // UNITY_APPLY_FOG(i.fogCoord, col);
    return col;
}

#endif
