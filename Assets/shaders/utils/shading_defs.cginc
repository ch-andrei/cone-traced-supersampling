#ifndef SHADING_DEFS_H
#define SHADING_DEFS_H

#define USE_SUN
#define USE_SUN_SDF
#define USE_AO
#define USE_SKY_SHADOW
#define USE_FOG

#define USE_CHECKERBOARD

#define FOG_COLOR float3(0.9, 0.9, 1.2)
#define FOG_FALLOFF 0.000015
#define FOG_START_DISTANCE 2.

#define SKY_COLOR float3(0.9, 0.9, 1.2)

#define SUN_COLOR float3(1.00, 0.8, 0.60)
#define SUN_DIR normalize(float3(-0.5, 0.4, -0.6))

#define CHECKERBOX_INTENSITY 0.85

#define MAX_SHADOW_STEPS 64  // max SDF marching steps for lights
#define MIN_SHADOW_STEP_SIZE 0.01
#define MAX_SHADOW_STEP_SIZE 0.25
#define MIN_SHADOW_TRACE_T 0.001  // minimum ray depth for tracing shadows
#define MAX_SHADOW_TRACE_T 5.  // maximum ray depth for tracing shadows

#endif
