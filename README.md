# SPPM

A simple but fast implementation of Stochastic Progressive Photon Mapping using c++.

Renders a Cornell Box scene with diffuse, mirror, glossy, and refractive materials.

## Build & Run

```bash
cmake -B build
cmake --build build
./build/RayTracing
```

Requires CMake 3.10+ and a C++17 compiler. Output PPM images (`binary.ppm`).

## Scene

500×500 Cornell Box. Camera at `(278, 273, -800)`, FOV 40°.

| Object | Material |
|--------|----------|
| Floor | White diffuse |
| Right wall | Green diffuse |
| Left wall | Mirror |
| Tall box | Red diffuse |
| Glass sphere (IOR 1.5) | Refractive |
| Ceiling | Area light |

## Algorithm — SPPM

Each of the 64 iterations:

1. **Camera rays** — cast one path per pixel; record a hit point at the first diffuse surface (mirror/glass bounce specularly).
2. **Hash grid** — insert hit points into a spatial hash for O(1) photon queries.
3. **Direct lighting** — one area-light sample + shadow ray per hit point.
4. **Photon tracing** — 100k photons from the light, bouncing with Russian roulette. Deposited only on diffuse/glossy surfaces within the current search radius.
5. **Radiance estimate** — $L = \frac{1}{\pi r^2} \sum \tau$, where $\tau$ is accumulated photon flux.
6. **Radius shrinkage** — $R_{i+1} = R_i \sqrt{(i + 0.67) / (i + 1)}$ ensures convergence.

Initial radius = 15, max photon depth = 10, gamma = 1/0.6.

## Architecture

```
photon_mapping/
├── main.cpp              # Scene setup
├── core/
│   ├── Vector.hpp/cpp    # 3D vector math
│   ├── Ray.hpp           # Ray (origin + direction)
│   ├── Bounds3.hpp       # AABB
│   ├── Intersection.hpp  # Ray-object hit record
│   └── global.hpp        # Constants, RNG, progress bar
├── object/
│   ├── Object.hpp        # Base interface
│   ├── Sphere.hpp        # Sphere primitive
│   ├── Triangle.hpp      # Triangle / MeshTriangle + OBJ loader
│   ├── Material.hpp      # BRDF: diffuse, mirror, glossy, refractive
│   ├── HitPoint.hpp      # Hit point struct + HashGrid spatial index
│   └── BVH.hpp/cpp       # Bounding Volume Hierarchy
├── scene/
│   ├── Light.hpp         # Light base
│   ├── AreaLight.hpp     # Area light sampling
│   └── Scene.hpp/cpp     # Scene + intersection
├── renderer/
│   └── Renderer.hpp/cpp  # SPPM multi-threaded render loop
└── models/               # OBJ meshes
```

## Key Features

- **Multi-threaded** — camera rays, direct lighting, and photon tracing all use `std::thread` with work stealing.
- **BVH** — SAH-based bounding volume hierarchy for fast ray-triangle intersection.
- **Hash grid** — spatial hashing for constant-time photon neighbour queries.
- **Russian roulette** — unbiased path termination based on surface albedo.
- **Fresnel** — full dielectric Fresnel for mirror and glass materials.
- **Progressive** — radius shrinks each iteration, converging to the correct radiance.

## References

- Hachisuka & Jensen (2009). Stochastic Progressive Photon Mapping. *SIGGRAPH Asia*.
- Jensen (2001). *Realistic Image Synthesis Using Photon Mapping*.
- Kajiya (1986). The Rendering Equation. *SIGGRAPH*.

## Original Homework Address

[GAMES101](https://games-cn.org/forums/topic/s2021-games101-zuoyehuizong/)
