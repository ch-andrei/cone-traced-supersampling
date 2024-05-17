#ifndef NORMALS_H
#define NORMALS_H

#define NORMAL_IMPL_EPS 0.0001

// 3 different normal calculation implementations
#define NORMAL_IMPL_UNITY 0  // 6 sdf calls, more precise
#define NORMAL_IMPL_UNITY_TETRAHYDRON 1 // 4 sdf calls, less precise
#define NORMAL_IMPL_SHADERTOY 2 // 4 sdf calls, less precise, Shadertoy compile optimization

#define NORMAL_IMPL NORMAL_IMPL_UNITY

#endif
