# ragdolls_featherstone

Video: https://youtu.be/G2soCTyrWgM

Ragdoll physics implementation from scratch in GDScript.

* Forces and impulses are propagated across the ragdoll using Featherstone's algorithm.
* Impulse-based collision response.




### Issues
* Interpenetration resolution doesn't work.
* The implementation is currently very unoptimized, with redundant matrix computations.




### Folder structure
```
.
├── addons
├── src
│   ├── articulated_body.gd # ragdoll physics implementation
│   ├── fps_label.gd
│   ├── grid.gdshader
│   ├── joint.gd
│   └── main.tscn           # main scene
└── addons
    ├── camera              # freelook camera addon
    ├── debug_draw_3d
    └── largelinearalgebra  # NxN matrices and utility functions
```
