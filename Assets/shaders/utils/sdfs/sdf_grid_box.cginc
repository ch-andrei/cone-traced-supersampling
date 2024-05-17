#ifndef SDF_GRID_BOX_H
#define SDF_GRID_BOX_H

#include "primitives.cginc"

#define GRID float3(0.2,100.,0.2)
#define GRID_TRACE

float rand(in float2 co)
{
    return frac(sin(dot(co, float2(1.29898, 7.8233))) * 437.585453);
}

float2 sdf(in float3 p)
{
    float2 res = float2(1e10, 0.0);

    res = opU(res, float2(sdPlane(p, 0.), 0.));

    float3 pr = opRep(p, GRID);
    float3 pi = opRepInd(p, GRID);

    float r = rand(pi.xz);
    res = opU(res, float2(sdRoundBox(pr, float3(0.35 * GRID.x, 0.15+0.2*r*r*r, 0.35 * GRID.z), 0.00), 100 * r));

    return res;
}
#endif
