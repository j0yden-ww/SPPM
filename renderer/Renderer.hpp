#pragma once

#include "Scene.hpp"
#include "HitPoint.hpp"

class Renderer
{
public:
    void Render(const Scene& scene);

private:
    void tracePhoton(const Scene& scene,
                     std::vector<HitPoint>& hitPoints,
                     const HashGrid& grid,
                     float totalEmitArea,
                     int numPhotonsPerPass,
                     float maxRadius,
                     std::vector<Vector3f>& fluxAccum,
                     std::vector<float>& M);

    Vector3f computeDirectIllumination(const Scene& scene,
                                       const Vector3f& hitPoint,
                                       const Vector3f& normal,
                                       const Vector3f& wo,
                                       const Vector3f& Kd,
                                       const Vector3f& Ks,
                                       float spExp);
};
