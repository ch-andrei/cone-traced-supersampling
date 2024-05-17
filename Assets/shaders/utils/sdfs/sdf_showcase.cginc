#ifndef SDF_SHOWCASE_H
#define SDF_SHOWCASE_H

#include "primitives_showcase.cginc"
#include "primitives_sponza.cginc"

float2 sdf(in float3 p)
{
    // // float2 res = sdf_showcase(p);
    // float2 res = 90009;
    //
    // float zoffset = 5;
    // float width = 2;
    // float dbox = sdBox(p - float3(0., width, zoffset + width), float3(width, width, width));
    // res = opU(dbox, res);
    //
    // if (dbox < 0.5)
    //     res = sdf_fast_sponza(float3(p.x, p.y, p.z)*2 - float3(0.,0.,zoffset));
    
    return sdf_showcase(p);
}

#endif
