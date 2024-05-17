#ifndef SDF_SINGLE_H
#define SDF_SINGLE_H

#include "primitives.cginc"

#define WHITE_BG

float2 sdf(in float3 p)
{
    float2 res = float2(1e10, 0.0);

    // res = opU(res, float2(sdPlane(p, 0.), 0.));

    // more primitives
    res = opU(res, float2(sdBox(p - float3(0.0, 1.0, 0.0), float3(1.0, 1.0, 1.0)), 46.23));

    return res;
}
#endif
