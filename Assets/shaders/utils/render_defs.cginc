#ifndef RENDER_DEFS_H
#define RENDER_DEFS_H

/* ---------------------------------------- */
// render parameters

#define MAX_AA_FACTOR 4
int _AA_FACTOR = 1;

// 1 for single sample, 2 for 2x2 supersampling, etc.
// #define AA_FACTOR 1
// #define AA_FACTOR 2
// #define AA_FACTOR 3

/* ---------------------------------------- */
// CTSS parameters

// #define NO_CTSS // disables CTSS

#define CTSS_NUM_SAMPLES 8 // maximum number of CTSS samples per pixel

#define USE_CTSS_WEIGHTED // toggle for using visibility-based weights; uncomment for uniform sample weights
// #define USE_CTSS_R // use CTSS Relaxed, faster approximation with sphere tracing termination at hard hit (not full hit)

// small value epsilon, precision for cone tracing intersections.
// Note: cone tracing eps=0.01 is similar to sphere tracing eps=0.0001
// min ~0.0005, else may have visual artifacts
#define OCCLUSION_EPS 0.01

#define OCCLUSION_SOFT (0. - OCCLUSION_EPS) // cone occlusion for soft hit detection
#define OCCLUSION_HARD (0.5 - OCCLUSION_EPS) // cone occlusion for hard hit detection

#ifdef USE_CTSS_R
#define OCCLUSION_STOP OCCLUSION_HARD // relaxed CTSS (faster, worse AA quality)
#else
#define OCCLUSION_STOP (1.0 - OCCLUSION_EPS) // full CTSS (slower, better AA quality)
#endif

#define MIN_SAMPLE_WEIGHT 0.01 // minimum allowed value for a soft hit group weight

#define SPHERE_TRACE_STEPSIZE_MULT 1.0 // controls step size as a multiple of SDF value, useful for non-ideal SDFs
#define CONE_RAD_STEP_MULT 0.5 // upper bound for the step size as a multiple of local cone Radius

/* ---------------------------------------- */

// #define USE_SUBPIXEL_EDGE_RESOLVE
// #define SUBPIXEL_EDGE_RESOLVE_CONE_MULT 2.0
// #define SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MAX 0.998
// #define SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MIN 0.9

bool _USE_SUBPIXEL_EDGE_RESOLVE = true;
bool _SUBPIXEL_EDGE_RESOLVE_TWO_PLANE_INTERSECTION = true;
float _SUBPIXEL_EDGE_RESOLVE_CONE_MULT_COS = 2.0;
float _SUBPIXEL_EDGE_RESOLVE_CONE_MULT = 1.0;
float _SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MAX = 0.999;
float _SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MIN = 0.9;
float _SUBPIXEL_EDGE_RESOLVE_CURVATURE_THRESHOLD = 0.00001;

/* ---------------------------------------- */
// Sphere tracing parameters

#define TMIN 0.01
#define TMAX 30.

#define MAX_RAYMARCH_STEPS 2048  // max SDF marching steps
float _MAX_RAYMARCH_STEPS = MAX_RAYMARCH_STEPS;  // max SDF marching steps

// cone radius expansion factor in units of pixel size
#define PIXEL_SIZE_MULT_INSCRIBED (1. / 2.) // cone is inscribed in the pixel (inside the pixel)
#define PIXEL_SIZE_MULT_CIRCUMSCRIBED (1. / sqrt(2.)) // cone is circumscribed in the pixel (intersects pixel corners)
#define PIXEL_SIZE_MULT_BALANCED (1. / ((2. + sqrt(2.)) / 2.)) // half way between inscribed and circumscribed
#define PIXEL_SIZE_MULT PIXEL_SIZE_MULT_BALANCED

/* ---------------------------------------- */

// CONE_BACKTRACE more correctly estimates the entry point of a cone hit as sphere tracing detects a delayed hit.
// With sphere tracing, the true cone intersection entry point is somewhere between iterations i-1 and i,
// but cone intersection is only detected at iteration i. This can lead to visual artifacts shading inside a surface.
// CONE_BACKTRACE gives a safe position between iterations i-1 and i, where no hit is guaranteed.
#define CONE_BACKTRACE_T(t, tanT) (t * (1. - tanT / (1. + tanT)))

// when defined, will backtrace and then run normal sphere tracing to determine a hard intersection
// #define CONE_BACKTRACE_SPHERE_TRACE

// maximum number of steps for sphere tracing after applying backtrace correction
#define CONE_BACKTRACE_SPHERE_TRACE_MAX_STEPS 32

/* ---------------------------------------- */

// boundary box parameters
// #define BBOX_FLOOR // toggle for using bbox

#define BBOX_FLOOR_Y -0.25 // y level of floor bbox

/* ---------------------------------------- */

// #define CAMERA_STATIC // disables camera panning

bool _CAMERA_STATIC = true;
float _CAMERA_SPIN = 0.025; // angle of each oscillations
float _CAMERA_SPEED = 0.05; // how many oscillations per second

/* ---------------------------------------- */

#define DEBUG_OPACITY 0.99

// #define DEBUG // flag to enable/disable all debugging

// #define DEBUG_NUM_SAMPLES
// #define DEBUG_DEPTH
// #define DEBUG_NORMAL
// #define DEBUG_STEPS

bool _DEBUG_SUBPIXEL_EDGE_RESOLVE_WEIGHT = false;
bool _DEBUG_SUBPIXEL_EDGE_RESOLVE_COSSIM = false;
bool _DEBUG_SUBPIXEL_EDGE_RESOLVE_DIRECTION = false;
bool _DEBUG_SUBPIXEL_EDGE_RESOLVE_CURVATURE = false;

#endif
