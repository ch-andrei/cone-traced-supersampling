#ifndef SHADING_H
#define SHADING_H

#include "sdf.cginc"
#include "shading_defs.cginc"

// http://iquilezles.org/www/articles/rmshadows/rmshadows.htm
float calcSoftshadow(in float3 ro, in float3 rd, in float mint, in float tmax)
{
    float res = 1.;
    float t = mint;
    for (int i = 0; i < MAX_SHADOW_STEPS; i++)
    {
        float3 p = ro + rd * t;
        float h = sdf(p).x;
        float s = clamp(8. * h / t, 0., 1.);
        res = min(res, s * s * (3. - 2. * s));

        #ifdef GRID
        float d2cell = abs(sd2dBox(opRep(p, GRID).xz, 0.5*GRID.xz));
        t += clamp(sign(h.x) * min(max(d2cell, 0.01), abs(h.x)), MIN_SHADOW_STEP_SIZE, MAX_SHADOW_STEP_SIZE); // mod for grid
        #else
        t += clamp(h, MIN_SHADOW_STEP_SIZE, MAX_SHADOW_STEP_SIZE);
        #endif

        if (res < 0.004 || t > tmax) break;
    }
    return clamp(res, 0., 1.);
}

#ifdef USE_AO
float calcAO(in float3 pos, in float3 nor)
{
    float occ = 0.;
    float sca = 1.;
    for (int i = 0; i < 5; i++)
    {
        float h = 0.01 + 0.12 * float(i) / 4.;
        float d = sdf(pos + h * nor).x;
        occ += (h - d) * sca;
        sca *= 0.95;
        if (occ > 0.35) break;
    }
    return clamp(1. - 3. * occ, 0., 1.) * (0.5 + 0.5 * nor.y);
}
#endif

// http://iquilezles.org/www/articles/checkerfiltering/checkerfiltering.htm
float checkersGradBox(in float2 p, in float2 dpdx, in float2 dpdy)
{
    // filter kernel
    float2 w = abs(dpdx) + abs(dpdy) + 0.001;
    // analytical integral (box filter)
    float2 i = 2. * (abs(frac((p - 0.5 * w) * 0.5) - 0.5) - abs(frac((p + 0.5 * w) * 0.5) - 0.5)) / w;
    // xor pattern
    return lerp(0.5 - 0.5 * i.x * i.y, 1., 1. - CHECKERBOX_INTENSITY);
    // return 1.;
}

float3 code2color( const in float m )
{
    return 0.2 + 0.2 * sin(m * 2. + float3(0., 1., 2.));
}

float3 shadedColor(
    in float3 ro, in float3 rd, in float3 rdx, in float3 rdy, in float m, in float3 pos, in float3 nor)
{
    // no hit
    if (m < -0.5)
        return float3(0., 0., 0.);

    float3 col = code2color(m);
    float ks = saturate(m / 10);

#ifdef USE_CHECKERBOARD
    if (m < 1.5)
    {
        // floor
        nor = float3(0., 1., 0.);
        // checker grad pattern
        // project pixel footprint into the plane
        float3 dpdx = ro.y * (rd / rd.y - rdx / rdx.y);
        float3 dpdy = ro.y * (rd / rd.y - rdy / rdy.y);
        col = checkersGradBox(3. * pos.xz, 3. * dpdx.xz, 3. * dpdy.xz);
        col = 0.1 + col * 0.05;
        ks = 1.0;
    }
#endif

#ifdef USE_AO
    // lighting
    float occ = calcAO(pos, nor);
#elif defined (SDF_M_IS_OCCLUSION)
    float occ = m;
#else
    float occ = 0.;
#endif

    float3 lin = float3(0., 0., 0.);

#ifdef USE_SUN
    // sun
    {
        float3 lig = SUN_DIR;
        float3 hal = normalize(lig - rd);
        float dif = saturate(dot(nor, lig));
#ifdef USE_SUN_SDF
        if( dif>0.0001 )
            dif *= calcSoftshadow(pos, lig, MIN_SHADOW_TRACE_T, MAX_SHADOW_TRACE_T);
#endif
        float spe = pow(saturate(dot(nor, hal)), 16.);
        spe *= dif;
        spe *= 0.04 + 0.96 * pow(saturate(1. - dot(hal, lig)), 5.);
        lin += col * 2.20 * dif * SUN_COLOR;
        lin += 5.00 * spe * SUN_COLOR * ks;
    }
#endif

#ifdef USE_SKY_SHADOW
    // sky
    {
        float3 ref = reflect(rd, nor);
        float dif = sqrt(clamp(0.5 + 0.5 * nor.y, 0., 1.));
        dif *= occ;
        float spe = smoothstep(-0.2, 0.2, ref.y);
        spe *= dif;
        spe *= 0.04 + 0.96 * pow(clamp(1. + dot(nor, rd), 0., 1.), 5.);
        //if( spe>0.001 )
        spe *= lerp(SKY_COLOR * calcSoftshadow(pos, ref, 0.005, MAX_SHADOW_TRACE_T), 1., 0.1);
        lin += col * 0.60 * dif * SKY_COLOR;
        lin += 1.0 * spe * SKY_COLOR * ks;
    }
#endif

    // back
    {
        float dif = clamp(dot(nor, normalize(float3(0.5, 0., 0.6))), 0., 1.) * clamp(
            1. - pos.y, 0., 1.);
        dif *= occ;
        lin += col * 0.6 * dif * float3(0.25, 0.25, 0.25);
    }
    // sss
    {
        float dif = pow(clamp(1. + dot(nor, rd), 0., 1.), 2.);
        dif *= occ;
        lin += col * 0.55 * dif * 1.;
    }

    col = lin;

#ifdef USE_FOG
    // fog
    float depth = length(ro - pos);
    float tf = clamp(depth - FOG_START_DISTANCE, 0., depth);
    col = lerp(col, FOG_COLOR, 1. - exp(-FOG_FALLOFF * tf * tf * tf));
#endif

    return col;
}
#endif
