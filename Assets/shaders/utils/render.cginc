#ifndef RENDER_H
#define RENDER_H

#include "sdf.cginc"
#include "shading.cginc"
#include "render_defs.cginc"

float3x3 setCamera(float3 ro, float3 ta, float cr)
{
    float3 cw = normalize(ta - ro);
    float3 cp = float3(sin(cr), cos(cr), 0.);
    float3 cu = normalize(cross(cw, cp));
    float3 cv = (cross(cu, cw));
    return float3x3(cu, cv, cw);
}

// returns float3(ray depth, object id, number of sphere tracing steps)
float3 sphereTrace(const float3 ro, const float3 rd, const float tan_theta, const int max_steps, const float tmin)
{
    float tmax = TMAX;
    #ifdef BBOX_FLOOR
    // raytrace "floor"
    float tp1 = (BBOX_FLOOR_Y - stp.ro.y) / stp.rd.y;
    if (tp1 > 0.)
    {
        tmax = min(stp.t_max, tp1);
    }
    #endif

    // sphere tracing
    float t = tmin;
    for (int i = 0; i < max_steps && t < tmax; i++)
    {
        float3 p = ro + rd * t; // current pos
        float2 h = sdf(p); // current sdf

        float coneRad = t * tan_theta;
        float coneOcclusion = (1. - h.x / coneRad) / 2.;

        // classical sphere tracing with cone intersection termination criterion
        if (OCCLUSION_HARD < coneOcclusion)
        {
            return float3(t, h.y, i); // ray depth and object code
        }

        // increment depth along ray
        float step = h.x * SPHERE_TRACE_STEPSIZE_MULT; // apply step size multiplier
#ifdef GRID_TRACE
        float d2cell = abs(sd2dBox(opRep(p, GRID).xz, 0.5 * GRID.xz));
        t += sign(step) * min(max(d2cell, 0.01), abs(step));
#else
        t += step;
#endif
    }

    return float3(t, -1., -1);
}

#ifdef NO_CTSS
float3 render(const float3 ro, const float3 rd, const float tan_theta, const float3 rdx, const float3 rdy)
{
    // compute intersection with SDFs
    // sample.x = ray depth, sample.y = object id
    float3 str = sphereTrace(ro, rd, tan_theta, MAX_RAYMARCH_STEPS, TMIN);

    // shade background if sphere tracing does not find intersection
    if (str.z < 0)
    {
#ifdef WHITE_BG
        return float3(1., 1., 1.);
#else
        return float3(clamp(FOG_COLOR - max(rd.y, 0.) * 0.3, 0., 1.));
#endif
    }

    // compute shading
    float3 pos = ro + rd * str.x;
    float3 nor = calcNormal(pos);
    float3 color = shadedColor(ro, rd, rdx, rdy, str.y, pos, nor);

#ifdef DEBUG
#ifdef DEBUG_NORMAL
    color = lerp(color, normal2color(nor), DEBUG_OPACITY);
#endif

// #define DEBUG_NORMAL_COS
#ifdef DEBUG_NORMAL_COS
    float coneRadius = str.x * tan_theta;
    float3 norCR = calcNormal(pos, 2.0 * coneRadius);
    float cosSim = dot(nor, norCR);

    if (cosSim < SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD)
    {
        ;
    } else
        color *= 0;
#endif

#ifdef DEBUG_STEPS
    float steps = float(str.z) / MAX_RAYMARCH_STEPS;
    color = lerp(color, steps, DEBUG_OPACITY);
#endif

#ifdef DEBUG_DEPTH
    float depth = t / TMAX;
    color = lerp(color, depth * depth, DEBUG_OPACITY);
#endif
#endif

    return color;
}
#else

#define TWO_OVER_FIVE_MINUS_ONE (2. / 5. - 1.)
#define FOUR_OVER_FIVE_MINUS_ONE (4. / 5. - 1.)
#define SIX_OVER_FIVE_MINUS_ONE (6. / 5. - 1.)
#define EIGHT_OVER_FIVE_MINUS_ONE (8. / 5. - 1.)
#define TEN_OVER_FIVE_MINUS_ONE (10. / 5. - 1.)

#define VISMASK_MIN_NORMAL 0.001f

#define SHIFT(x, n) ((0 <= n) ? (x << n): (x >> -n))

typedef struct
{
    float3 position;
    float t_in;
    float t_out;
    float t_s;
    float id;
    float occlusion;
} sample_ctss;

/**
    Returns a 32-bit visibility mask given current cone radius, SDF value, and SDF normal.
    Mask represents ~6x6 subpixel visibility (occupancy).
    Visibility over 6 horizontal lines is encoded with 4, 6, 6, 6, 6, 4 bits (32 bits total).
    Note that corner bits are omitted since 6x6=36, but we only have 32 bits.
    Example bit mask with full visibility (Xs show the space that is omitted in the mask):
        X1111X
        111111
        111111
        111111
        111111
        X1111X
    This will be packed in a single int as 32 bits: 1111 111111 111111 111111 111111 1111
*/
int getVisibilityMask(float coneOcclusion, float2 nor)
{
    // ensure non-zero normal x, y components
    float2 n = normalize(float2(
        clamp(abs(nor.x), VISMASK_MIN_NORMAL, 1.) * (nor.x < 0. ? -1.: 1.),
        clamp(abs(nor.y), VISMASK_MIN_NORMAL, 1.) * (nor.y < 0. ? -1.: 1.)
    ));  // clamp abs then apply sign

    float h = (1. - 2. * coneOcclusion);
    h = -h;

    // define y = ax + b to describe how the surface intersects the traced cone
    // solve for a and b using simple geometry and math
    float a = -n.x / n.y;  // slope;
    float b = h * (n.y * n.y + n.x * n.x) / n.y;  // y intersect

    // NOTE:
    // yIncr = 2. / 5. * r;
    // for i in (0, 1, 2, 3, 4, 5)
    //     py = yIncr * i - r
    //     vis = ((py - b) / a / r + 1.) / 2. = (py - b) / a / r / 2. + 0.5
    float t = 0.5f / a;

    // solve for x (intersection of surface and horizontal line given by y)
    // saturate visibility to 0-1
    float vis0 = saturate((-1.0 - b) * t + 0.5);
    float vis1 = saturate((TWO_OVER_FIVE_MINUS_ONE - b) * t + 0.5);
    float vis2 = saturate((FOUR_OVER_FIVE_MINUS_ONE - b) * t + 0.5);
    float vis3 = saturate((SIX_OVER_FIVE_MINUS_ONE - b) * t + 0.5);
    float vis4 = saturate((EIGHT_OVER_FIVE_MINUS_ONE - b) * t + 0.5);
    float vis5 = saturate((TEN_OVER_FIVE_MINUS_ONE - b) * t + 0.5);

    // apply vis - 1 if n.x is positive
    float vis_r = (0 < n.x) ? -1.: 0.; // 0 or -1
    // convert visibility to bit shift amounts (max 6 bit shift)
    int shift0 = (int)(6. * (vis0 + vis_r));
    int shift1 = (int)(6. * (vis1 + vis_r));
    int shift2 = (int)(6. * (vis2 + vis_r));
    int shift3 = (int)(6. * (vis3 + vis_r));
    int shift4 = (int)(6. * (vis4 + vis_r));
    int shift5 = (int)(6. * (vis5 + vis_r));

    int mask_int = 0;  // 32-bit mask
    int mask4 = 0x1E; // 0b00011110, the side bits are 0s for line 0 and 5
    int mask6 = 0x3F; // 0b00111111;  // all 6 bits are used for lines 1, 2, 3, 4
    mask_int |= (SHIFT(mask6, shift0) & mask4) << 27;  // >> 1 << 28 = << 27; 1 extra right shift (since mask4)
    mask_int |= (SHIFT(mask6, shift1) & mask6) << 22;
    mask_int |= (SHIFT(mask6, shift2) & mask6) << 16;
    mask_int |= (SHIFT(mask6, shift3) & mask6) << 10;
    mask_int |= (SHIFT(mask6, shift4) & mask6) << 4;
    mask_int |= (SHIFT(mask6, shift5) & mask4) >> 1;  // >> 1 << 0 = >> 1; 1 extra right shift (since mask4)

    return mask_int;
}

int bitCountOnes(int num) {
    int count = 0;
    for (int i = 0; i < 32; i++)
    {
        count += num & 1;
        num = num >> 1;
    }
    return count;
}

#ifdef USE_SUBPIXEL_EDGE_RESOLVE
float resolve_primary_edge_visibility(const float3 n1, const float3 n2, float h1, float h2,
    const float3 rd, const float3 searchDir, float coneRadius)
{
    // note: for all local projections, x axis is along searchDir, y axis is along rd
    // searchDir and rd are orthogonal

    // local projection of the normals of the two planes
    float np1rd = dot(-rd, n1);
    float np2rd = dot(-rd, n2);
    float np1sd = dot(searchDir, n1);
    float np2sd = dot(searchDir, n2);
    // 2d projected normals (without normalization)
    float2 n1p = float2(np1sd, np1rd);
    float2 n2p = float2(np2sd, np2rd);

    // local points on the two planes
    // h is distance along projected 3d normal
    float2 p1 = - h1 * n1p; // origin - h1p * n1p
    float2 p2 = float2(coneRadius, 0.0) - h2 * n2p;
    // normalize direction only after computing p1 and p2
    n1p = normalize(n1p);
    n2p = normalize(n2p);

    // describe lines as ax - y = -b
    // a = -n.x / n.y because normal is orthogonal to line
    // b = y - a x
    // line 1
    float a1 = -n1p.x / n1p.y;
    float b1 = p1.y - a1 * p1.x;
    // line 2
    float a2 = -n2p.x / n2p.y;
    float b2 = p2.y - a2 * p2.x;

    // determine the intersection point of the two lines
    // we only need the x coordinate
    float xi = (b1 - b2) / (a2 - a1); // local x (along searchDir) position

    // the intersection points controls the visibility of the primary edge sample
    return (clamp(xi, -coneRadius, coneRadius) / coneRadius + 1.0) / 2.0;
}
#endif

float3 render(const float3 ro, const float3 rd, const float tan_theta, const float3 rdx, const float3 rdy)
{
    // setup total color and total weight
    float3 colorTotal = float3(0., 0., 0.);
    float weightTotal = 0.;

    // record sample information during sphere tracing
#ifndef USE_SUBPIXEL_EDGE_RESOLVE
    sample_ctss samples[CTSS_NUM_SAMPLES];
#else
    sample_ctss samples[CTSS_NUM_SAMPLES + 1]; // +1 sample to allow for edge resolve
#endif

    int numSamples = 0;
    bool hasHardHit = false; // ensure that only the first hard hit is sampled (for each group)

#ifdef USE_CTSS_WEIGHTED
    int visibilityMask = 0;
#endif

    // previous values for hasHit, hardHit, and sdf call
    bool hasHitP = false; // previous
    bool hardHitP = false; // previous
    float2 hP; // previous ray depth and object code

    bool hasFullHit = false;

    // sphere tracing
    float t = TMIN; // current t
    float tP = t; // initialize previous t as t

    float t_max = TMAX;
#ifdef BBOX_FLOOR
    // raytrace floor plane
    float tp1 = (BBOX_FLOOR_Y - ro.y) / rd.y;
    if (tp1 > 0.)
    {
        t_max = min(t_max, tp1);
    }
#endif

    int numSteps = 0;
    for (; numSteps < _MAX_RAYMARCH_STEPS && t < t_max && numSamples < CTSS_NUM_SAMPLES; numSteps++)
    {
        float3 p = ro + rd * t;
        float2 h = sdf(p);

        float coneRad = t * tan_theta;
        float coneOcclusion = (1. - h.x / coneRad) / 2.;

        bool hasHit = OCCLUSION_SOFT < coneOcclusion;
        bool hardHit = OCCLUSION_HARD < coneOcclusion;
        bool fullHit = OCCLUSION_STOP < coneOcclusion;

        bool hitEntry = hasHit && !hasHitP;
        bool hitExit = !hasHit && hasHitP;
        bool hardHitEntry = hardHit && !hardHitP;

        // update previous hit 
        hasHitP = hasHit;
        hardHitP = hardHit;

        if (hasHit)
        {
            if (hitEntry)
            {
                hasHardHit = false;

                // init new sample, record hit group entry point
                // used backtraced intersection: current ray depth minus safe cone radius
                float t_entry = CONE_BACKTRACE_T(t, tan_theta);
                // initialize t in, t out, t sample
                samples[numSamples].t_in = t_entry; 
                samples[numSamples].t_out = t_entry;
                samples[numSamples].t_s = t_entry;
                samples[numSamples].position = ro + rd * t_entry; // sdf
                samples[numSamples].id = h.y; // id
#ifdef USE_CTSS_WEIGHTED
                samples[numSamples].occlusion = coneOcclusion;
#endif
            }

#ifdef USE_CTSS_WEIGHTED
            // update maximal cone occlusion for the current group
            samples[numSamples].occlusion = max(samples[numSamples].occlusion, coneOcclusion);
#endif

            // if hit group has a hard hit, record the first hard hit
            if (hardHitEntry && !hasHardHit)
            {
                hasHardHit = true; // record that a hard hit was found for the current hit group

                // record hard hit intersection
#ifdef CONE_BACKTRACE_SPHERE_TRACE
                // if sphere trace correction is enabled,
                // sphere_trace from (previous ray depth + previous SDF value)
                // TODO: fix this for new ctss_sample struct
                float3 str_h = sphereTrace(ro, rd, tan_theta, CONE_BACKTRACE_SPHERE_TRACE_MAX_STEPS, tP + hP.x);
                samples[numSamples].xy = str_h.xy; // sdf, id
#else
                // fallback to previous ray depth + previous SDF value (safe position)
                float t_s = tP + hP.x;
                samples[numSamples].t_s = t_s; // sample position
                samples[numSamples].position = ro + rd * t_s; // sdf
                samples[numSamples].id = hP.y; // id
#endif
            }
        }

        // record the sample only when hit group exit is detected or a full hit occurs
        if (hitExit || fullHit)
        {
            // record group exit position
            samples[numSamples].t_out = CONE_BACKTRACE_T(t, tan_theta);

            // reset control vars and increment sample counter
            numSamples++;
        }

        if (fullHit)
        {
            hasFullHit = true;
            break;
        }

        // update previous h and t
        hP = h;
        tP = t;

        float step = h.x * SPHERE_TRACE_STEPSIZE_MULT;

        // increment depth along ray
#ifdef GRID_TRACE
        // if on a grid with randomized objects (for each cell),
        // need to cap maximum move distance to distance from the point within cell to the boundary of the cell,
        // otherwise, the trace may miss geometry and have visual artifacts
        float d2cell = abs(sd2dBox(opRep(p, GRID).xz, 0.5 * GRID.xz));
        t += max(min(max(d2cell, 0.01), abs(step)), CONE_RAD_STEP_MULT * coneRad);
#else
        t += max(step, CONE_RAD_STEP_MULT * coneRad); // speed up sphere tracing near hard hits (sdf ~= 0)
#endif
    }

#if defined(USE_CTSS_WEIGHTED) || defined(USE_SUBPIXEL_EDGE_RESOLVE) 
    // for normal projection, assuming camera is level with the horizon
    // compute right and up vectors given the camera ray rd
    float3 right = normalize(cross(rd, float3(0., 1., 0.)));
    float3 up = normalize(cross(right, rd));
#endif
    
#ifdef USE_SUBPIXEL_EDGE_RESOLVE
    bool hasSecondaryEdge = false;
    float3 normalSecondaryEdge = float3(0.,0.,0.);
    float2 normal2dResolvedEdge = float2(0.,0.);

    // read the last sample and apply subpixel edge resolve for full occlusion samples
    sample_ctss sample = samples[numSamples - 1];
#ifdef DEBUG
    if (_USE_SUBPIXEL_EDGE_RESOLVE && OCCLUSION_STOP < sample.occlusion)
#else
    if (OCCLUSION_STOP < sample.occlusion)
#endif
    {
        // calculate small scale normal (the normal of the primary edge)
        float coneRadius = sample.t_s * tan_theta;
        // compute small scale normal
        float3 normalPrimaryEdge = calcNormal(sample.position);
        // compute large scale normal (instead of h=epsilon, use h=Rc, where Rc is the current cone radius)
        float4 normalLarge = calcNormalH4(sample.position, _SUBPIXEL_EDGE_RESOLVE_CONE_MULT_COS * coneRadius);

        // compute cosine similarity between small and large scale normals
        float normalSimPrimary = dot(normalPrimaryEdge, normalLarge.xyz);
        float curvature = normalLarge.w;

        // if normals disagree by more than threshold,
        // then pixel contains an edge and we will apply subpixel edge resolve 
        hasSecondaryEdge = normalSimPrimary < _SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MAX &&
            _SUBPIXEL_EDGE_RESOLVE_CURVATURE_THRESHOLD < abs(curvature);

 #ifdef DEBUG
        if (_DEBUG_SUBPIXEL_EDGE_RESOLVE_COSSIM && !_DEBUG_SUBPIXEL_EDGE_RESOLVE_CURVATURE)
        {
            float sim = 1.0 - normalSimPrimary;
            return float3(sim, sim, sim);
        }

        if (_DEBUG_SUBPIXEL_EDGE_RESOLVE_CURVATURE)
        {
            if (!_DEBUG_SUBPIXEL_EDGE_RESOLVE_COSSIM)
            {
                float curv = (abs(curvature) < _SUBPIXEL_EDGE_RESOLVE_CURVATURE_THRESHOLD) ? 0: sign(curvature);
                curv = (1.0 + curv) / 2.0;
                return float3(curv, curv, curv);
            }
            else if (!hasSecondaryEdge)
            {
                return float3(0, 0, 0);
            }
#endif

        if (hasSecondaryEdge)
        {
            // locate the secondary edge side.
            // the sign of the curvature describes local edge as convex/concave,
            // which in turn controls the direction towards the secondary edge.
            // Direction is {from normalPrimaryEdge to normalLarge} for convex,
            // and {from normalLarge to normalPrimaryEdge} for concave edges;
            // curvature is positive for convex and negative for concave edges.
            float3 searchDir = sign(curvature) * normalize(normalLarge - normalPrimaryEdge);
            // project the diff direction from world space to camera space
            normal2dResolvedEdge = normalize(float2(dot(right, searchDir), dot(up, searchDir)));
            // reproject the 2d diff direction back to 3d
            searchDir = normalize(normal2dResolvedEdge.x * right + normal2dResolvedEdge.y * up);

            // resolve the secondary side position
            float3 resolvedPosition = sample.position + searchDir * _SUBPIXEL_EDGE_RESOLVE_CONE_MULT * coneRadius;
            // evaluate normal for secondary side
            normalSecondaryEdge = calcNormal(resolvedPosition);
            // evaluate sdf for secondary side (may be different object id)
            float2 h2 = sdf(resolvedPosition);
            // offset position by sdf and normal
            resolvedPosition = resolvedPosition - normalSecondaryEdge * h2.x * (1.0 - sign(h2.x) * OCCLUSION_EPS);

#ifdef DEBUG
            if (_DEBUG_SUBPIXEL_EDGE_RESOLVE_CURVATURE)
            {
                if (_DEBUG_SUBPIXEL_EDGE_RESOLVE_COSSIM)
                {
                    float curv = (1.0 + sign(curvature)) / 2.0;
                    return float3(curv, 1 - curv, 0);
                }
            }

            if (_DEBUG_SUBPIXEL_EDGE_RESOLVE_DIRECTION)
            {
                return normal2color(searchDir);
            }

            // debug mode allows toggle for plane-plane or small-large normals heuristic
            float occlusion;
            if (_SUBPIXEL_EDGE_RESOLVE_TWO_PLANE_INTERSECTION)
            {
                float2 h1 = sdf(sample.position);
                occlusion = resolve_primary_edge_visibility(normalPrimaryEdge, normalSecondaryEdge, h1.x, h2.x, rd,
                    searchDir, _SUBPIXEL_EDGE_RESOLVE_CONE_MULT * coneRadius);
            }
            else
            {
                // compute the half angle between the two normals
                float3 halfAngle = (normalPrimaryEdge + normalSecondaryEdge) / length(normalPrimaryEdge + normalSecondaryEdge);
                float halfAngleToLargeSim = dot(normalLarge.xyz, halfAngle);
                // estimate visibility of the primary edge using the similarity between large scale and half angle normals 
                occlusion = 1.0 - 0.5 * saturate(remapGlsl(
                    _SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MIN,
                    _SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MAX,
                    0.0,
                    1.0,
                    halfAngleToLargeSim
                    ));
            }

            if (_DEBUG_SUBPIXEL_EDGE_RESOLVE_WEIGHT)
            {
                return float3(occlusion, occlusion, occlusion);
            }
#else
            float2 h1 = sdf(sample.position);
            float occlusion = resolve_primary_edge_visibility(normalPrimaryEdge, normalSecondaryEdge, h1.x, h2.x, rd,
                searchDir, _SUBPIXEL_EDGE_RESOLVE_CONE_MULT * coneRadius);
#endif

            // update primary sample visibility
            samples[numSamples - 1].occlusion = occlusion;

            // add secondary sample
            sample_ctss resolved_sample;
            resolved_sample.t_in = sample.t_in;
            resolved_sample.t_out = sample.t_out;
            resolved_sample.t_s = length(resolvedPosition - ro);
            resolved_sample.position = resolvedPosition;
            resolved_sample.id = h2.y; // this may be different from the primary edge
            resolved_sample.occlusion = 1.0; // full visibility, since secondary side will be sampled after primary side
            samples[numSamples] = resolved_sample;
            numSamples++;
        }
    }
#endif

    // compute shading samples
    for (int i = 0; i < numSamples; i++)
    {
        sample_ctss sample = samples[i];
        float3 nor = calcNormal(sample.position);
#ifdef USE_SUBPIXEL_EDGE_RESOLVE
        if (hasSecondaryEdge && (i == numSamples - 1)) // only the secondary side (last sample)
            nor = normalSecondaryEdge;
#endif

#ifdef USE_CTSS_WEIGHTED
        // project the normal from world space to camera space
        float2 nor2d = normalize(float2(dot(right, nor), dot(up, nor)));
#ifdef USE_SUBPIXEL_EDGE_RESOLVE
        if (hasSecondaryEdge && (i == numSamples - 2)) // only the primary side (second to last sample)
            nor2d = normal2dResolvedEdge;
#endif

        // compute current visibility mask given pixel projected normal and group's maximum cone occlusion
        int visibilityMaskCrt = getVisibilityMask(sample.occlusion, nor2d);
        visibilityMaskCrt &= ~visibilityMask; // correlation with previous hits, removes invisible bits
        visibilityMask |= visibilityMaskCrt; // update visibility mask given current visibility, adds visible bits

        float visibility = bitCountOnes(visibilityMaskCrt) / 32.; // visible bit ratio
        float weight = max(MIN_SAMPLE_WEIGHT, visibility);
#else
        float weight = 1.0;
#endif

        float3 color = shadedColor(ro, rd, rdx, rdy, sample.id, sample.position, nor);

        colorTotal += weight * color;
        weightTotal += weight;
    }

    // background (sky)
    if (!hasFullHit)
    {
#ifdef USE_CTSS_WEIGHTED
        float bgWeight = 1.0 - bitCountOnes(visibilityMask) / 32.0;
#else
        float bgWeight = 1.0;
#endif

#ifdef WHITE_BG
        colorTotal += bgWeight * float3(1., 1., 1.);
#else
        colorTotal += bgWeight * float3(clamp(FOG_COLOR - max(rd.y, 0.) * 0.3, 0., 1.));
#endif
        weightTotal += bgWeight;
    }

    colorTotal /= max(MIN_SAMPLE_WEIGHT, weightTotal);

#ifdef DEBUG
#ifdef DEBUG_STEPS
    float steps = float(numSteps) / MAX_RAYMARCH_STEPS;
    colorTotal = lerp(colorTotal, steps, DEBUG_OPACITY);
#endif

#ifdef DEBUG_DEPTH
    float depth = t / TMAX;
    colorTotal = lerp(colorTotal, depth * depth, DEBUG_OPACITY);
#endif

#ifdef DEBUG_NUM_SAMPLES
    float v = float(numSamples) / float(CTSS_NUM_SAMPLES + 1);
    colorTotal = lerp(colorTotal, v * v, DEBUG_OPACITY);
#endif
#endif
        
    return colorTotal;
}
#endif

float3 pixelColor(float2 pixel, float3 ro, float3 ta, float tan_theta, float focal_length)
{
    float3 color = float3(0., 0., 0.);

    // camera-to-world transformation
    float3x3 ca = setCamera(ro, ta, 0.);

    // ray differentials
    float2 px = (2. * (pixel + float2(1., 0.)) - _ScreenParams.xy) / _ScreenParams.y;
    float2 py = (2. * (pixel + float2(0., 1.)) - _ScreenParams.xy) / _ScreenParams.y;
    float3 rdx = mul(normalize(float3(px, focal_length)), ca);
    float3 rdy = mul(normalize(float3(py, focal_length)), ca);

    int aa_factor = (int)(clamp(_AA_FACTOR, 1.0, MAX_AA_FACTOR));

    // #if AA>1
    for( int m=0; m<aa_factor; m++ )
    for( int n=0; n<aa_factor; n++ )
    {
        // pixel coordinates
        float2 o = float2(float(m), float(n)) / float(aa_factor) - (0.5 - 1. / float(2 * aa_factor));
        float2 p = (2. * (pixel + o) - _ScreenParams.xy) / _ScreenParams.y;
    // #else
        // float2 p = (2. * pixel - _ScreenParams.xy) / _ScreenParams.y;
    // #endif

        float3 fw = normalize(float3(p, focal_length));

        // ray direction
        float3 rd = mul(fw, ca);

        color += render(ro, rd, tan_theta, rdx, rdy);
    // #if AA>1
    }
    color /= float(aa_factor*aa_factor);
    // #endif

    // gain
    color = color * 3. / (2.5 + color);

    // gamma
    color = pow(color, float3(0.4545, 0.4545, 0.4545));

    return color;
}

#endif
