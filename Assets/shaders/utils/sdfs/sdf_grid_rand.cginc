#ifndef SDF_GRID_RAND_H
#define SDF_GRID_RAND_H

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
    int obj = fmod(int(100 * r), 10); // fmod(r, 10) vs fmod(r, 50) for sparse
    if (obj == 5)
        res = opU(
            res,
            float2(sdRoundBox(pr, float3(0.25 * GRID.x, 0.05+0.2*r*r*r, 0.25 * GRID.z), 0.025), 100 * r)
            );
    else if (obj == 6)
        res = opU(
            res,
            float2(sdBox(pr, float3(0.45 * GRID.x, 0.05+0.2*r*r*r, 0.45 * GRID.z)), 100 * r)
            );
    else if (obj == 7)
        res = opU(
            res,
            float2(sdSphere(pr-float3(0.,0.2*GRID.x,0.), 0.45*GRID.x), 100 * r)
            );
    else if (obj == 8)
        res = opU(
            res,
            float2(sdCylinder(pr, float2(0.4 * GRID.x, 0.05+0.2*r*r*r)), 100 * r)
            );
    else {
        // empty cell
    }

    return res;
}
#endif
