#ifndef SDF_H
#define SDF_H

/* SDF params and functions */

#include "scenes.cginc"
#include "math.cginc"

#if SCENE == SCENE_SHOWCASE
#include "sdfs/sdf_showcase.cginc"
#elif SCENE == SCENE_PRIMITIVES
#include "sdfs/sdf_primitives.cginc"
#elif SCENE == SCENE_GRID_RAND
#include "sdfs/sdf_grid_rand.cginc"
#elif SCENE == SCENE_GRID_BOX
#include "sdfs/sdf_grid_box.cginc"
#elif SCENE == SCENE_SINGLE
#include "sdfs/sdf_single.cginc"
#elif SCENE == SCENE_SPONZA
#include "sdfs/sdf_sponza.cginc"
#elif SCENE == SCENE_SPONZA_SHOWCASE
#include "sdfs/sdf_sponza_showcase.cginc"
#else
#include "sdfs/sdf_primitives.cginc"
#endif

/* ---------------------------------------- */

#include "sdfs/normal_defs.cginc"

/* ---------------------------------------- */

float4 calcNormalH4( float3 pos, const float h ) // for function f(p)
{
    // same as the base implementation below, but also returns the sum of all tested SDF values minus non-zero h0 offset
    const float2 k = float2(h, 0);
    float h0 = sdf(pos).x;
    float h1 = sdf(pos + k.xyy).x;
    float h2 = sdf(pos - k.xyy).x;
    float h3 = sdf(pos + k.yxy).x;
    float h4 = sdf(pos - k.yxy).x;
    float h5 = sdf(pos + k.yyx).x;
    float h6 = sdf(pos - k.yyx).x;
    return float4(normalize( float3(h1 - h2, h3 - h4, h5 - h6)), h1 + h2 + h3 + h4 + h5 + h6 - 6.0 * h0);
}

// https://iquilezles.org/articles/normalsSDF/
#if NORMAL_IMPL==NORMAL_IMPL_UNITY
float3 calcNormalH( float3 pos, const float h ) // for function f(p)
{
    const float2 k = float2(h, 0);
    return normalize( float3(sdf(pos + k.xyy).x - sdf(pos - k.xyy).x,
                             sdf(pos + k.yxy).x - sdf(pos - k.yxy).x,
                             sdf(pos + k.yyx).x - sdf(pos - k.yyx).x ) );
}
#elif NORMAL_IMPL==NORMAL_IMPL_UNITY_TETRAHYDRON
// assumes that we are computing normal at the surface boundary: sdf(pos) = 0, zero iso-surface of the SDF 
float3 calcNormalH( float3 pos, const float h ) // for function f(p)
{
    const float2 k = float2(1., -1.);
    return normalize( k.xyy * sdf( pos + k.xyy * h ).x +
                      k.yyx * sdf( pos + k.yyx * h ).x +
                      k.yxy * sdf( pos + k.yxy * h ).x +
                      k.xxx * sdf( pos + k.xxx * h ).x );
}
#elif NORMAL_IMPL==NORMAL_IMPL_SHADERTOY
float3 calcNormalH( float3 pos, const float h )
{
    // inspired by tdhooper and klems - a way to prevent the compiler from inlining map() 4 times
    float3 n = float3(0., 0., 0.);
    for (int i = 0; i < 4; i++)
    {
        float3 e = 0.5773 * (2. * float3((((i + 3) >> 1) & 1), ((i >> 1) & 1), (i & 1)) - 1.);
        n += e * sdf(pos + h * e).x;
        //if( n.x+n.y+n.z>100. ) break;
    }
    return normalize(n);
}
#endif

float3 calcNormal( float3 pos )
{
    return calcNormalH(pos, NORMAL_IMPL_EPS);
}

float3 calcNormal( float3 pos, const float h )
{
    return calcNormalH(pos, max(NORMAL_IMPL_EPS, h)); // limit minimal value for h
}

float3 normal2color( float2 normal )
{
    return float3((normal + 1.) / 2., 0);
}

float3 normal2color( float3 normal )
{
    return (normal + 1.) / 2.;
}

#endif
