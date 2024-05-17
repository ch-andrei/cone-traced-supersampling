#ifndef SDF_SPONZA_H
#define SDF_SPONZA_H

#include "primitives_sponza.cginc"

float2 sdf(in float3 p)
{
    return sdf_fast_sponza(p);
}

#endif
