#include <sstream>
#include <iomanip>
#include <thread>
#include <vector>
#include "Scene.hpp"
#include "Renderer.hpp"

// 角度转弧度
inline float deg2rad(const float& deg) { return deg * M_PI / 180.0f; }

const float EPSILON = 0.00001f;

// ---- 工具函数 -------------------------------------------------------

// 并行 for 循环：将 [0, total) 均匀分给各线程
template<typename Func>
inline void parallelFor(int total, int numThreads, Func&& func) {
    int perThread = (total + numThreads - 1) / numThreads;
    std::vector<std::thread> threads;
    for (int t = 0; t < numThreads; t++) {
        int start = t * perThread;
        int end = std::min(start + perThread, total);
        if (start >= total) break;
        threads.emplace_back([start, end, &func]() {
            for (int i = start; i < end; i++) func(i);
        });
    }
    for (auto& t : threads) t.join();
}

// 写出 PPM 图像（含 gamma 校正）
inline void savePPM(const char* filename, const std::vector<Vector3f>& framebuffer,
                    int width, int height, int iterationCount) {
    FILE* fp = fopen(filename, "wb");
    if (!fp) return;
    fprintf(fp, "P6\n%d %d\n255\n", width, height);
    float inv = 1.0f / (float)iterationCount;
    for (int i = 0; i < width * height; i++) {
        Vector3f avg = framebuffer[i] * inv;
        unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, avg.x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, avg.y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, avg.z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}

// ===================================================================
// computeDirectIllumination — 直接光照（面光源采样 + shadow ray）
// ===================================================================
Vector3f Renderer::computeDirectIllumination(
    const Scene& scene, const Vector3f& hitPoint,
    const Vector3f& normal, const Vector3f& wo,
    const Vector3f& Kd, const Vector3f& Ks, float spExp)
{
    Vector3f L_dir(0.0f);

    Intersection lightInter;
    float pdf_light = 0.0f;
    scene.sampleLight(lightInter, pdf_light);

    if (pdf_light > EPSILON) {
        Vector3f dirToLight = lightInter.coords - hitPoint;
        float dist = dirToLight.norm();
        Vector3f wi = normalize(dirToLight);

        const float shadowBias = 0.001f;
        Vector3f shadowOrigin = hitPoint + normal * shadowBias;
        Ray shadowRay(shadowOrigin, wi);
        Intersection shadowInter = scene.intersect(shadowRay);

        // 通畅条件：未命中 / 命中光源 / 命中物体在光源后方
        bool hitIsLight = shadowInter.happened &&
                          shadowInter.m != nullptr &&
                          shadowInter.m->hasEmission();
        bool hitBeyondLight = shadowInter.distance > dist - shadowBias;

        if (!shadowInter.happened || hitIsLight || hitBeyondLight) {
            // 漫反射 BRDF + Blinn-Phong 高光
            Vector3f f_r = Kd * (1.0f / M_PI);

            if (spExp > 0.0f && (Ks.x > 0.0f || Ks.y > 0.0f || Ks.z > 0.0f)) {
                Vector3f halfVec = normalize(wi + wo);
                float specCos = std::max(0.0f, dotProduct(halfVec, normal));
                float spec = std::pow(specCos, spExp);
                f_r = f_r + Ks * (spExp + 2.0f) / (2.0f * M_PI) * spec;
            }

            float cosTheta = std::max(0.0f, dotProduct(normal, wi));
            float cosThetaPrime = std::max(0.0f, dotProduct(lightInter.normal, -wi));

            // L = L_e × f_r × cos(theta) × cos(theta)' / r^2 / pdf
            L_dir = lightInter.emit * f_r * cosTheta * cosThetaPrime
                    / std::max(dist * dist, 1e-6f) / pdf_light;
        }
    }

    return L_dir;
}

// ===================================================================
// tracePhoton — 发射并追踪一枚光子
// ===================================================================
void Renderer::tracePhoton(const Scene& scene,
                           std::vector<HitPoint>& hitPoints,
                           const HashGrid& grid,
                           float totalEmitArea,
                           int numPhotonsPerPass,
                           float maxRadius,
                           std::vector<Vector3f>& fluxAccum,
                           std::vector<float>& M)
{
    const int maxPhotonDepth = 10;
    const float bias = 0.001f;

    // 在光源表面随机发射光子
    Intersection lightInter;
    float pdf_light = 0;
    scene.sampleLight(lightInter, pdf_light);
    if (pdf_light < EPSILON) return;

    // 余弦加权出射方向，匹配漫反射光源分布
    Vector3f wi = sampleCosineHemisphere(lightInter.normal);

    // 单光子通量 = 光源总通量 / 光子数
    // 总通量 = L_e × A × pi（pi 来自半球积分 cal{cos(theta) d(omega)}）
    Vector3f power = lightInter.emit * totalEmitArea * M_PI / (float)numPhotonsPerPass;
    Vector3f photonPos = lightInter.coords + lightInter.normal * bias;

    std::vector<int> nearby;

    for (int depth = 0; depth < maxPhotonDepth; depth++) {
        Ray ray(photonPos, wi);
        Intersection inter = scene.intersect(ray);

        if (!inter.happened) break;
        if (inter.m->hasEmission()) break;

        Vector3f N = inter.normal;
        Vector3f p = inter.coords;

        // 镜面反射：不沉积，完美反射后继续
        if (inter.m->m_type == MIRROR) {
            wi = inter.m->sample(wi, N);
            power = power * inter.m->Ks;
            photonPos = p + N * bias;
            continue;
        }

        // 折射：不沉积，Fresnel 轮盘赌决定反射/折射后继续
        if (inter.m->m_type == REFRACTIVE) {
            wi = inter.m->sample(wi, N);
            power = power * inter.m->Ks;
            photonPos = p + wi * bias;   // 折射时沿出射方向偏移
            continue;
        }

        // Glossy：轮盘赌选择镜面反射或漫反射沉积
        if (inter.m->m_type == GLOSSY) {
            float kd_avg = (inter.m->Kd.x + inter.m->Kd.y + inter.m->Kd.z) / 3.0f;
            float ks_avg = (inter.m->Ks.x + inter.m->Ks.y + inter.m->Ks.z) / 3.0f;
            float totalRefl = kd_avg + ks_avg;
            if (totalRefl < EPSILON) break;

            float probSpecular = ks_avg / totalRefl;
            if (get_random_float() < probSpecular) {
                wi = wi - 2.0f * dotProduct(wi, N) * N;
                power = power * inter.m->Ks / probSpecular;
                photonPos = p + N * bias;
                continue;
            } else {
                power = power / (1.0f - probSpecular);
            }
        }

        // 光子沉积
        grid.query(p, maxRadius, hitPoints, nearby);
        for (int idx : nearby) {
            HitPoint& hp = hitPoints[idx];
            float cos_i = -dotProduct(hp.normal, wi);
            if (cos_i <= 0) continue;
            fluxAccum[idx] += power * hp.Kd * (1.0f / M_PI);
            M[idx]++;
        }

        // 俄罗斯轮盘赌
        float p_survive = std::min(1.0f,
            std::max({inter.m->Kd.x, inter.m->Kd.y, inter.m->Kd.z}));
        if (get_random_float() > p_survive) break;

        // 余弦加权散射方向 + 功率更新
        // power = power × Kd/pi × cos(theta) / (p_survive × cos((theta)/pi) = power × Kd / p_survive
        wi = sampleCosineHemisphere(N);
        power = power * inter.m->Kd / p_survive;
        photonPos = p + N * bias;
    }
}

// ===================================================================
// Render — SPPM 主循环（多线程）
// ===================================================================
void Renderer::Render(const Scene& scene)
{
    // ---- 参数 ----
    const int   numIterations     = 64;
    const int   numPhotonsPerPass = 100000;
    const float initialRadius     = 15.0f;
    const float alpha             = 0.67f;

    const int numThreads = std::max(1u, std::thread::hardware_concurrency());

    // 光源总面积（用于光子功率）
    float totalEmitArea = 0;
    for (auto* obj : scene.get_objects()) {
        if (obj->hasEmit()) totalEmitArea += obj->getArea();
    }

    std::vector<Vector3f> framebuffer(scene.width * scene.height, Vector3f(0));

    // 相机：位于 (278, 273, -800)，FOV=40度，看向 +z
    float scale = tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    const int maxCameraBounce = 10;
    const float rayBias = 0.001f;

    float globalRadius = initialRadius;
    std::vector<HitPoint> hitPoints(scene.width * scene.height);

    // ================================================================
    // 迭代循环
    // ================================================================
    for (int iter = 0; iter < numIterations; iter++) {

        for (auto& hp : hitPoints) {
            hp.valid = false;
            hp.M = 0;
            hp.fluxAccum = Vector3f(0);
        }

        // 步骤 1: 相机光线生成
        parallelFor(scene.height, numThreads, [&](int j) {
            int m = j * scene.width;
            for (int i = 0; i < scene.width; i++) {
                // 像素内随机抖动
                float jx = get_random_float();
                float jy = get_random_float();

                float x = (2 * (i + jx) / (float)scene.width - 1)
                          * imageAspectRatio * scale;
                float y = (1 - 2 * (j + jy) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                Ray ray(eye_pos, dir);
                Vector3f throughput(1.0f);

                for (int bounce = 0; bounce < maxCameraBounce; bounce++) {
                    Intersection inter = scene.intersect(ray);

                    if (!inter.happened) {
                        if (bounce == 0)
                            framebuffer[m] = framebuffer[m] + scene.backgroundColor;
                        break;
                    }

                    if (inter.m->hasEmission()) {
                        framebuffer[m] = framebuffer[m]
                                       + inter.m->getEmission() * throughput;
                        break;
                    }

                    if (inter.m->m_type == MIRROR || inter.m->m_type == REFRACTIVE) {
                        Vector3f wi = inter.m->sample(
                            normalize(ray.direction), inter.normal);
                        throughput = throughput * inter.m->Ks;
                        ray = Ray(inter.coords + wi * rayBias, wi);
                        continue;
                    }

                    HitPoint& hp = hitPoints[m];
                    hp.position  = inter.coords;
                    hp.normal    = inter.normal;
                    hp.wo        = normalize(-ray.direction);
                    hp.Kd        = inter.m->Kd * throughput;
                    hp.Ks        = inter.m->Ks * throughput;
                    hp.spExp     = inter.m->specularExponent;
                    hp.radius    = globalRadius;
                    hp.valid     = true;
                    break;
                }
                m++;
            }
        });

        // 步骤 2: 哈希网格
        HashGrid hashGrid(globalRadius);
        hashGrid.build(hitPoints);

        // 步骤 3: 直接光照
        parallelFor((int)hitPoints.size(), numThreads, [&](int k) {
            if (!hitPoints[k].valid) return;
            HitPoint& hp = hitPoints[k];
            Vector3f L_dir = computeDirectIllumination(
                scene, hp.position, hp.normal, hp.wo, hp.Kd, hp.Ks, hp.spExp);
            framebuffer[k] = framebuffer[k] + L_dir;
        });

        // 步骤 4: 光子追踪（各线程独立累积，最后合并）
        {
            int photonsPerThread = numPhotonsPerPass / numThreads;
            int remainder = numPhotonsPerPass % numThreads;

            std::vector<std::vector<Vector3f>> threadFluxAccum(numThreads);
            std::vector<std::vector<float>> threadM(numThreads);
            for (int t = 0; t < numThreads; t++) {
                threadFluxAccum[t].resize(hitPoints.size(), Vector3f(0));
                threadM[t].resize(hitPoints.size(), 0);
            }

            std::vector<std::thread> threads(numThreads);
            for (int t = 0; t < numThreads; t++) {
                int myPhotons = photonsPerThread + (t == numThreads - 1 ? remainder : 0);
                threads[t] = std::thread([&, t, myPhotons]() {
                    for (int p = 0; p < myPhotons; p++) {
                        tracePhoton(scene, hitPoints, hashGrid,
                                    totalEmitArea, numPhotonsPerPass, globalRadius,
                                    threadFluxAccum[t], threadM[t]);
                    }
                });
            }
            for (auto& t : threads) t.join();

            for (int k = 0; k < (int)hitPoints.size(); k++) {
                if (!hitPoints[k].valid) continue;
                for (int t = 0; t < numThreads; t++) {
                    hitPoints[k].fluxAccum += threadFluxAccum[t][k];
                    hitPoints[k].M += threadM[t][k];
                }
            }
        }

        // 步骤 5: 辐射度估计 L = fluxAccum / (pi × r^2)
        parallelFor((int)hitPoints.size(), numThreads, [&](int k) {
            if (!hitPoints[k].valid || hitPoints[k].M == 0) return;
            float r2 = globalRadius * globalRadius;
            Vector3f L_photon = hitPoints[k].fluxAccum / (M_PI * r2);
            framebuffer[k] = framebuffer[k] + L_photon;
        });

        // 步骤 6: 半径缩小 R_{i+1} = R_i × sqrt((i+alpha)/(i+1))
        globalRadius *= std::sqrt((iter + alpha) / (iter + 1.0f));

        UpdateProgress((float)(iter + 1) / numIterations);

        // 中间输出
        if ((iter + 1) % 8 == 0 || iter == numIterations - 1) {
            std::ostringstream fn;
            fn << "iter_" << std::setw(3) << std::setfill('0')
               << (iter + 1) << ".ppm";
            savePPM(fn.str().c_str(), framebuffer, scene.width, scene.height, iter + 1);
        }
    }

    UpdateProgress(1.f);

    // 最终图像
    savePPM("binary.ppm", framebuffer, scene.width, scene.height, numIterations);
}
