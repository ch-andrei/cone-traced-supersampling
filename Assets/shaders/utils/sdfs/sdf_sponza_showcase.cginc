#ifndef SPONZA_SHOWCASE_H
#define SPONZA_SHOWCASE_H

#include "primitives_showcase.cginc"
#include "primitives_sponza.cginc"

float2 sdf(in float3 p)
{
    // for better lighting, remove front wall
    // float zd = p.z - 12;
    // if (0.1 < zd)
        // return zd;

    float2 prim = sdf_showcase((p - float3(-0.8, 0.001, 7.25)));
    float2 sponza = sdf_fast_sponza(p);

    return opU(prim, sponza);
}

#endif
