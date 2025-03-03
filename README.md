# ragdolls_featherstone

Video: https://youtu.be/G2soCTyrWgM

Ragdoll physics implementation from scratch in GDScript.

* Forces and impulses are propagated across the ragdoll using Featherstone's algorithm.
* Impulse-based collision response.




### Issues
* Interpenetration resolution doesn't work.
* The implementation is currently very unoptimized, with redundant matrix computations and in GDScript instead of C++.


### Todo
* ~~Precompute frequently used matrices~~
* ~~Rewrite the implementation in C++~~
* Experiment with interpenetration resolution code
    * Currently it's calculating the velocity required to move the bodies apart by "depth of collision", fictitiously applying the corresponding impulse, and then applying the resulting velocities with a fake delta time of 1 sec.
    * Try treating the whole ragdoll as a single rigid body for the purposes of interpenetration resolution.
    * Try implementing rollback to prevent interpenetration altogether.
    * Read up more on interpenetration resolution.


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
