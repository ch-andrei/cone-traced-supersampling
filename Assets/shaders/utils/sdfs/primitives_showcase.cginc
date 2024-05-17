#ifndef PRIMITIVES_SHOWCASE_H
#define PRIMITIVES_SHOWCASE_H

// NOTE: THE ORIGINAL LICENSE FOR THE SDF CODE WITHOUT CTSS
//
// The MIT License
// Copyright © 2013 Inigo Quilez
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// A list of useful distance function to simple primitives. All
// these functions (except for ellipsoid) return an exact
// euclidean distance, meaning they produce a better SDF than
// what you'd get if you were constructing them from boolean
// operations.

// List of other 3D SDFs: https://www.shadertoy.com/playlist/43cXRl
// and http://iquilezles.org/www/articles/distfunctions/distfunctions.htm

#include "primitives.cginc"

float2 sdf_showcase(in float3 p)
{
    float2 res = float2(100000, 0);

    p = float3(p.x, p.y, -p.z);
    
    res = opU(res, float2(sdPlane(p, 0.), 0.));

    // bounding box
    if (sdBox(p - float3(0.0, 0.3, -1.0), float3(0.35, 0.3, 2.5)) < res.x)
    {
        // more primitives
        res = opU(res, float2(sdBoundingBox(p - float3(0.0, 0.25, 0.0), float3(0.3, 0.25, 0.2), 0.025),
                              16.9));
        res = opU(res, float2(sdTorus((p - float3(0.0, 0.30, 1.0)).xzy, float2(0.25, 0.05)), 25.0));
        res = opU(res, float2(sdCone(p - float3(0.0, 0.45, -1.0), float2(0.6, 0.8), 0.45), 55.0));
        res = opU(res, float2(sdCappedCone(p - float3(0.0, 0.25, -2.0), 0.25, 0.25, 0.1), 13.67));
        res = opU(res, float2(sdSolidAngle(p - float3(0.0, 0.00, -3.0), float2(3, 4) / 5.0, 0.4), 49.13));
        res = opU(res, float2(sdSphere(p - float3(0, 0.25, -0), 0.15), 26.9));
    }

    // bounding box
    if (sdBox(p - float3(1.0, 0.3, -1.0), float3(0.35, 0.3, 2.5)) < res.x)
    {
        // more primitives
        res = opU(res, float2(sdCappedTorus((p - float3(1.0, 0.30, 1.0)) * float3(1, -1, 1),
                                            float2(0.866025, -0.5), 0.25, 0.05), 8.5));
        res = opU(res, float2(sdBox(p - float3(1.0, 0.25, 0.0), float3(0.3, 0.25, 0.1)), 3.0));
        res = opU(res, float2(sdCapsule(p - float3(1.0, 0.00, -1.0), float3(-0.1, 0.1, -0.1),
                                        float3(0.2, 0.4, 0.2), 0.1), 31.9));
        res = opU(res, float2(sdCylinder(p - float3(1.0, 0.25, -2.0), float2(0.15, 0.25)), 8.0));
        res = opU(res, float2(sdHexPrism(p - float3(1.0, 0.2, -3.0), float2(0.2, 0.05)), 18.4));
    }

    // bounding box
    if (sdBox(p - float3(-1.0, 0.35, -1.0), float3(0.35, 0.35, 2.5)) < res.x)
    {
        // more primitives
        res = opU(res, float2(sdPyramid(p - float3(-1.0, -0.6, -3.0), 1.0), 13.56));
        res = opU(res, float2(sdOctahedron(p - float3(-1.0, 0.15, -2.0), 0.35), 23.56));
        res = opU(res, float2(sdTriPrism(p - float3(-1.0, 0.15, -1.0), float2(0.3, 0.05)), 43.5));
        res = opU(res, float2(sdEllipsoid(p - float3(-1.0, 0.25, 0.0), float3(0.2, 0.25, 0.05)), 43.17));
        res = opU(res, float2(sdRhombus((p - float3(-1.0, 0.34, 1.0)).xzy, 0.15, 0.25, 0.04, 0.08),
                              12.5));
    }

    // bounding box
    if (sdBox(p - float3(2.0, 0.3, -1.0), float3(0.35, 0.3, 2.5)) < res.x)
    {
        // more primitives
        res = opU(res, float2(sdOctogonPrism(p - float3(2.0, 0.2, -3.0), 0.2, 0.05), 51.8));
        res = opU(res, float2(sdCylinder(p - float3(2.0, 0.15, -2.0), float3(0.1, -0.1, 0.0),
                                         float3(-0.2, 0.35, 0.1), 0.08), 31.2));
        res = opU(res, float2(sdCappedCone(p - float3(2.0, 0.10, -1.0), float3(0.1, 0.0, 0.0),
                                           float3(-0.2, 0.40, 0.1), 0.15, 0.05), 46.1));
        res = opU(res, float2(sdRoundCone(p - float3(2.0, 0.15, 0.0), float3(0.1, 0.0, 0.0),
                                          float3(-0.1, 0.35, 0.1), 0.15, 0.05), 51.7));
        res = opU(res, float2(sdRoundCone(p - float3(2.0, 0.20, 1.0), 0.2, 0.1, 0.3), 37.0));
    }

    return res;
}
#endif
