//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth > this->maxDepth)
        return Vector3f(0.0f);

    Intersection inter = intersect(ray);

    if (!inter.happened)
        return Vector3f(0.0f);

    // Hit a light source — only count emission for primary rays
    if (inter.m->hasEmission()) {
        if (depth == 0)
            return inter.m->getEmission();
        else
            return Vector3f(0.0f);
    }

    Vector3f p = inter.coords;
    Vector3f N = inter.normal;
    Vector3f wo = -ray.direction;  // direction towards camera
    Material* m = inter.m;

    // === Direct Lighting ===
    Vector3f L_dir(0.0f);

    Intersection lightInter;
    float pdf_light = 0.0f;
    sampleLight(lightInter, pdf_light);

    if (pdf_light > EPSILON) {
        Vector3f dirToLight = lightInter.coords - p;
        float dist = dirToLight.norm();
        Vector3f wi = normalize(dirToLight);

        // Bias shadow ray origin to avoid self-intersection.
        // Use a larger epsilon (1e-3) than the global EPSILON (1e-5)
        // and bias along both the normal and the light direction
        // to handle grazing angles robustly.
        const float shadowBias = 0.001f;
        Vector3f shadowOrigin = p + N * shadowBias;
        Ray shadowRay(shadowOrigin, wi);
        Intersection shadowInter = intersect(shadowRay);

        // Unblocked if: no hit, or the hit object is emissive (the light itself),
        // or the hit distance is approximately at/behind the light.
        bool hitIsLight = shadowInter.happened &&
                          shadowInter.m != nullptr &&
                          shadowInter.m->hasEmission();
        bool hitBeyondLight = shadowInter.distance > dist - shadowBias;
        if (!shadowInter.happened || hitIsLight || hitBeyondLight) {
            Vector3f f_r = m->eval(wo, wi, N);
            float cosTheta = std::max(0.0f, dotProduct(N, wi));
            float cosThetaPrime = std::max(0.0f, dotProduct(lightInter.normal, -wi));
            L_dir = lightInter.emit * f_r * cosTheta * cosThetaPrime
                    / std::max(dist * dist, 1e-6f) / pdf_light;
        }
    }

    // === Indirect Lighting (Russian Roulette) ===
    Vector3f L_indir(0.0f);

    if (get_random_float() < this->RussianRoulette) {
        Vector3f wi = m->sample(wo, N);
        Ray indirectRay(p + N * 0.001f, wi);
        Intersection indirectInter = intersect(indirectRay);

        if (indirectInter.happened && !indirectInter.m->hasEmission()) {
            float pdf = m->pdf(wo, wi, N);
            if (pdf > EPSILON) {
                Vector3f f_r = m->eval(wo, wi, N);
                float cosTheta = std::max(0.0f, dotProduct(N, wi));
                L_indir = castRay(indirectRay, depth + 1)
                          * f_r * cosTheta / std::max(pdf, 1e-6f)
                          / this->RussianRoulette;
            }
        }
    }

    return L_dir + L_indir;
}