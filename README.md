# Cone-Traced Supersampling with Subpixel Edge Reconstruction

## Contents

This repository contains a Unity3D project showcasing our HLSL implementation of cone-traced supersampling (CTSS) and subpixel edge reconstruction (SER).

A quad is drawn on the screen with the SDFs defined as a material shader.

For SDF-based scenes, see ./Assets/shaders.

For CTSS/SER parameters and code, see ./Assets/shaders/utils/render.cginc and ./Assets/shaders/utils/render_defs.cginc

We also have an interactive demo of CTSS and SER at Shadertoy (our Unity3D code ported to GLSL): https://www.shadertoy.com/view/7lSXWK

## Citations

If you find our work useful, please cite as

```
@ARTICLE{chubarau2024ctss_ser,
  author={Chubarau, Andrei and Zhao, Yangyang and Rao, Ruby and Nowrouzezahrai, Derek and Kry, Paul G.},
  journal={IEEE Transactions on Visualization and Computer Graphics}, 
  title={Cone-Traced Supersampling with Subpixel Edge Reconstruction}, 
  year={2023},
  volume={},
  number={},
  pages={1-12},
  keywords={Rendering (computer graphics);Image edge detection;Shape;Image reconstruction;Real-time systems;Image quality;Geometry;Signed Distance Fields;Cone Tracing;Antialiasing;Rendering;Computer Graphics},
  doi={10.1109/TVCG.2023.3343166}}

@inproceedings{chubarau2023ctss,
  title={Cone-Traced Supersampling for Signed Distance Field Rendering},
  author={Andrei Chubarau and Yangyang Zhao and Ruby Rao and Paul Kry and Derek Nowrouzezahrai},
  booktitle={Graphics Interface 2023},
  year={2023},
  url={https://openreview.net/forum?id=FYhiH9IyBq}
}
```
