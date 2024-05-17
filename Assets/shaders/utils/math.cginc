#ifndef MATH_H
#define MATH_H

float dot2( in float2 v ) { return dot(v,v); }
float dot2( in float3 v ) { return dot(v,v); }
float ndot( in float2 a, in float2 b ) { return a.x*b.x - a.y*b.y; }

float invLerp(float from, float to, float value)
{
    return (value - from) / (to - from);
}

float remapGlsl(float origFrom, float origTo, float targetFrom, float targetTo, float value)
{
    float rel = invLerp(origFrom, origTo, value);
    return lerp(targetFrom, targetTo, rel);
}

float3 opRepInd( in float3 p, in float3 c )
{
    return floor((p+0.5*c)/c)-0.5*c;
}

float3 opRep( in float3 p, in float3 s )
{
    float3 ps = p+s*0.5;
    return sign(ps)*fmod(ps,s)-s*0.5;  // sign is needed to work in all quadrants
}

float2 opU( float2 d1, float2 d2 )
{
    return (d1.x<d2.x) ? d1 : d2;
}

// polynomial smooth min
float opSmoothMin2( float a, float b )
{
    float k = 2.;
    float h = max( k-abs(a-b), 0.0 )/k;
    return min( a, b ) - h*h*k*(1.0/4.0);
}
#endif
