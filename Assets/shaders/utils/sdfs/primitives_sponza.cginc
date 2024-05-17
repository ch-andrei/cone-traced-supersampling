// Code from mmerchante Sponza SDF implementation:
// https://www.shadertoy.com/view/ltGcRW
// https://www.shadertoy.com/view/XddBD2

#ifndef PRIMITIVES_SPONZA_H
#define PRIMITIVES_SPONZA_H

float glslMod(float x, float y)
{
    return x - y * floor(x/y);
}

// hg
float vmax(float3 v) {
    return max(max(v.x, v.y), v.z);
}

// hg
float fBox(float3 p, float3 b) {
    float3 d = abs(p) - b;
    return length(max(d, float3(0.,0.,0.))) + vmax(min(d, float3(0.,0.,0.)));
}

// hg
float vmax(float2 v) {
    return max(v.x, v.y);
}

// hg
float fCylinder(float3 p) {
    float d = length(p.xz) - .5;
    d = max(d, abs(p.y) - 1.0);
    return d;
}

float domainRepeat1D(float p, float size)
{
    return glslMod(abs(p) + size * .5, size) - size * .5;
}

// hg
float2 pModPolar(float2 p, float repetitions) {
    float angle = 2.0 * 3.1415 / repetitions;
    float a = atan(p.y/p.x) + angle/2.;
    float r = length(p);
    float c = floor(a/angle);
    a = glslMod(a, angle) - angle/2.;
    return float2(cos(a), sin(a))*r;
}

// hg
void pR(inout float2 p, float a) {
    p = cos(a)*p + sin(a)*float2(p.y, -p.x);
}

float3 rdX(float3 p)
{
    return float3(p.x, p.z, -p.y);
}

float3 rdY(float3 p)
{
    return float3(-p.z, p.y, p.x);
}

float3 rdZ(float3 p)
{
    return float3(-p.y, p.x, p.z);
}

// hg
float fCylinder(float3 p, float r, float height) {
    float d = length(p.xz) - r;
    d = max(d, abs(p.y) - height);
    return d;
}

// A shorter, uglier but faster version of https://www.shadertoy.com/view/XddBD2
float2 sdf_fast_sponza(in float3 p)
{
    // y bound
    if (13.75 < p.y)
        return float2(1., 0.);

    p.y -= 2.0;
    float3 a0 = p;
    a0.xz = abs(a0.xz) * float2(-1.0,1.0);
    float3 a1 = a0 - float3(6.24,.0,2.5);
    a1.xz = pModPolar(a1.xz, 4.0);
    float d1 = -(a1 - float3(11.49,.0,.0)).x;
    float3 a2 = a1 - float3(11.02,2.15,7.28);
    a2.z = domainRepeat1D(a2.z, 2.0);
    float d3 = fBox(a2 - float3(-2.64,5.05,.0),float3(.5,.5,.228));
    d3 = min(d3,fBox(a2 - float3(-2.275,5.05,.0),float3(.383,.383,.175)));
    d3 = min(d3,fBox(a2 - float3(-2.64,6.97,.0),float3(.5,.283,.111)));
    float d2 = max(-d3,fBox(a2 - float3(-1.28,6.38,.287),float3(1.5,1.893,6.673)));
    float3 a4 = a1 - float3(9.18,-4.5,-.032);
    a4.y = domainRepeat1D(a4.y, 4.5);
    float3 a5 = float3(a4.x, a4.y, domainRepeat1D(a4.z, 2.5));
    float3 a6 = float3(-a5.x, a5.y, a5.z);
    float3 a8 = rdZ(a6 - float3(.05,-.62,.0));
    float d8 = (fCylinder(a8, 1.398,1.361)*.75);
    d8 = max(-d8,(fCylinder(a8 - float3(.0,.152,.0), 1.434,.531)*.75));
    float d7 = max(d8,fBox(a6 - float3(.786,.46,.0),float3(.523,.747,1.415)));
    float d9 = fBox(a6 - float3(.47,1.953,.0),float3(.5,.075,1.5));
    d9 = min(d9,fBox(a6 - float3(.58,2.2,.0),float3(.5,.1,1.5)));
    d9 = min(d9,fBox(a6 - float3(-.45,-2.3,.0),float3(1.5,.1,1.5)));
    float3 a10 = a6 - float3(.463,-.51,1.179);
    a10.z = domainRepeat1D(a10.z, 2.35);
    float d10 = fBox(a10,float3(.24,.033,.24));
    d10 = min(d10,fBox(a10 - float3(.0,-.093,.0),float3(.24,.033,.24)));
    d10 = min(d10,fBox(a10 - float3(-2.8,-.03,.0),float3(.25,.075,.25)));
    float3 a11 = float3(a10.y, pModPolar(a10.xz , 8.0)).yxz;
    float d11 = fBox(a11 - float3(.002,-1.07,.0),float3(.17,1.053,.424));
    float3 a12 = a6 - float3(-1.03,-.518,.0);
    float3 a13 = rdZ(a12);
    float d13 = fCylinder(float3(a13.x, -a13.z, a13.y), 1.225,3.0);
    d13 = min(d13,fCylinder(a13, 1.094,2.061));
    float d12 = max(-d13,fBox(a12 - float3(.12,1.27,.0),float3(1.5,1.355,1.551)));
    float3 a14 = a6 - float3(.463,1.57,1.61);
    float d14 = fCylinder(float3(a14.y, -a14.x, a14.z) - float3(-.19, -.13, -1.08), .105,.046);
    float3 polePos = float3(-a14.y, a14.x, a14.z) - float3(.042, .596, -1.08);
    polePos.xy += 0.3428 * float2(polePos.y, -polePos.x);
    d14 = min(d14,fCylinder(polePos, .025,.582));
    float dout = min(min(min(d1,d2),min(min(d7,min(min(d9,min(d10,d11)),d12)),d14)),(a0 - float3(.0,-2.0,.0)).y);

    return float2(dout, 4.3);
}

#endif
