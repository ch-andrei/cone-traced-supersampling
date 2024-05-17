Shader "Unlit/primitives"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _MousePos ("MousePos", Vector) = (0.,0.,0.,0.)
        
        _AA_FACTOR ("AA factor N for NxN grid (supported 1x1, 2x2, 3x3)", Int) = 1

        _MAX_RAYMARCH_STEPS ("Sphere Tracing Max Steps", Int) = 2048

        [Toggle] _CAMERA_STATIC("Static camera", Float) = 1
        _CAMERA_SPEED ("CAMERA_SPEED", Range(0.0001, 1.0)) = 0.05
        _CAMERA_SPIN ("CAMERA_SPIN", Range(0.0001, 1.0)) = 0.025

        _SUBPIXEL_EDGE_RESOLVE_CONE_MULT ("SER_CONE_MULT", Range(0.01, 10.0)) = 1.0
        _SUBPIXEL_EDGE_RESOLVE_CONE_MULT_COS ("SER_CONE_MULT_COS", Range(0.01, 10.0)) = 2.0
        _SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MAX ("SER_COSSIM_THRESHOLD_MAX", Range(0.0, 1.0)) = 0.999
        _SUBPIXEL_EDGE_RESOLVE_COSSIM_THRESHOLD_MIN ("SER_COSSIM_THRESHOLD_MIN", Range(0.0, 1.0)) = 0.8
        _SUBPIXEL_EDGE_RESOLVE_CURVATURE_THRESHOLD ("SER_CURVATURE_THRESHOLD", Range(0.0, 0.01)) = 0.00001

        [Toggle] _USE_SUBPIXEL_EDGE_RESOLVE ("Use SER", Float) = 1
        [Toggle] _SUBPIXEL_EDGE_RESOLVE_TWO_PLANE_INTERSECTION ("Resolve subpixel plane-plane intersection visibility", Float) = 1

        [Toggle] _DEBUG_SUBPIXEL_EDGE_RESOLVE_WEIGHT ("DEBUG WEIGHT", Float) = 0
        [Toggle] _DEBUG_SUBPIXEL_EDGE_RESOLVE_COSSIM ("DEBUG COSSIM", Float) = 0
        [Toggle] _DEBUG_SUBPIXEL_EDGE_RESOLVE_DIRECTION ("DEBUG DIRECTION", Float) = 0
        [Toggle] _DEBUG_SUBPIXEL_EDGE_RESOLVE_CURVATURE ("DEBUG CURVATURE", Float) = 0
    }
    SubShader
    {
        Tags
        {
            "RenderType"="Opaque"
        }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            //------------------------------------------------------------------

            // Note:
            // Modified by Anonymous Authors (2023) based on code for SDF primitives showcase by Inigo Quilez
            // Original license from Inigo Quilez:

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

            //------------------------------------------------------------------

            // vertex and fragment shaders
            #include "utils/vertex_fragment.cginc"

            #include "utils/scenes.cginc"
            #define SCENE SCENE_PRIMITIVES

            #include "utils/shading_defs.cginc"
            #undef FOG_FALLOFF
            #define FOG_FALLOFF 0.0001
            #undef FOG_START_DISTANCE
            #define FOG_START_DISTANCE 1.

            #include "utils/render.cginc"

            // define a focal length for this scene
            #define FOCAL_LENGTH 3.5

            //------------------------------------------------------------------

            float4 _MousePos;

            float4 mainImage(in float2 pixel)
            {
                float time = 32.0 + _Time.y * 0.25;
                float t = (0.5 + 0.5 * sin(2. * 6.28 * _CAMERA_SPEED * time)) * clamp(_CAMERA_SPIN, 0., 1.);
                if (_CAMERA_STATIC)
                    t *= 0.0;

                float2 mo = _MousePos.xy + float2(0.4, 0.2);

                float3 ta = float3(0.5, 0.1, -0.8);
                float3 ro = ta + 5.0 * float3(sin(6.28 * (mo.x + t)), 1.0 * mo.y, cos(6.28 * (mo.x + t)));

                // NOTE: sqrt(2. * dp * dp) = sqrt(2.) * dp; // dp = half pixel size
                // NOTE: (sqrt(2.) * PIXEL_SIZE_MULT) modifies multiplier for pixel diagonal
                float tan_theta = 2. * PIXEL_SIZE_MULT / _ScreenParams.y / FOCAL_LENGTH;

                // render
                float3 col = pixelColor(pixel, ro, ta, tan_theta, FOCAL_LENGTH);

                return float4(col, 1.0);
            }

            ENDCG
        }
    }
}
