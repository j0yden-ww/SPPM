#pragma once

#include "Vector.hpp"
#include "global.hpp"
#include <vector>
#include <unordered_map>
#include <cmath>

// Cosine-weighted hemisphere sampling around a normal.
// Returns a unit direction in the hemisphere defined by N.
inline Vector3f sampleCosineHemisphere(const Vector3f& N) {
    float x_1 = get_random_float(), x_2 = get_random_float();
    float z = std::fabs(1.0f - 2.0f * x_1);
    float r = std::sqrt(1.0f - z * z);
    float phi = 2 * M_PI * x_2;
    Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);

    // Build local-to-world transform from N
    Vector3f B, C;
    if (std::fabs(N.x) > std::fabs(N.y)) {
        float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
        C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
    } else {
        float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
        C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
    }
    B = crossProduct(C, N);
    return localRay.x * B + localRay.y * C + localRay.z * N;
}

struct HitPoint {
    Vector3f position;
    Vector3f normal;
    Vector3f wo;           // direction to camera
    Vector3f Kd;           // diffuse albedo
    Vector3f Ks;           // specular albedo (for GLOSSY material)
    float   spExp;         // specular exponent (for GLOSSY material)
    Vector3f flux;         // accumulated corrected flux τ
    Vector3f fluxAccum;    // per-pass flux accumulator Δτ
    Vector3f directSum;    // accumulated direct illumination (sum over passes)
    float radius;          // current search radius
    float N;               // accumulated photon count (weighted, float)
    float  M;              // kernel-weighted photon count in current pass
    bool   valid;

    HitPoint()
        : radius(0), N(0), M(0), valid(false), spExp(0) {}
};

class HashGrid {
public:
    float cellSize;
    std::unordered_map<uint64_t, std::vector<int>> cells;

    HashGrid(float size) : cellSize(size) {}

    uint64_t hash(int x, int y, int z) const {
        return  ((uint64_t)(unsigned int)x * 73856093ull) ^
                ((uint64_t)(unsigned int)y * 19349663ull) ^
                ((uint64_t)(unsigned int)z * 83492791ull);
    }

    void build(const std::vector<HitPoint>& hitPoints) {
        cells.clear();
        for (int i = 0; i < (int)hitPoints.size(); i++) {
            if (!hitPoints[i].valid) continue;
            int cx = (int)std::floor(hitPoints[i].position.x / cellSize);
            int cy = (int)std::floor(hitPoints[i].position.y / cellSize);
            int cz = (int)std::floor(hitPoints[i].position.z / cellSize);
            cells[hash(cx, cy, cz)].push_back(i);
        }
    }

    void query(const Vector3f& photonPos, float maxRadius,
               const std::vector<HitPoint>& hitPoints,
               std::vector<int>& result) const {
        int minX = (int)std::floor((photonPos.x - maxRadius) / cellSize);
        int maxX = (int)std::floor((photonPos.x + maxRadius) / cellSize);
        int minY = (int)std::floor((photonPos.y - maxRadius) / cellSize);
        int maxY = (int)std::floor((photonPos.y + maxRadius) / cellSize);
        int minZ = (int)std::floor((photonPos.z - maxRadius) / cellSize);
        int maxZ = (int)std::floor((photonPos.z + maxRadius) / cellSize);

        result.clear();
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    auto it = cells.find(hash(x, y, z));
                    if (it == cells.end()) continue;
                    for (int idx : it->second) {
                        const HitPoint& hp = hitPoints[idx];
                        float dx = hp.position.x - photonPos.x;
                        float dy = hp.position.y - photonPos.y;
                        float dz = hp.position.z - photonPos.z;
                        if (dx * dx + dy * dy + dz * dz < hp.radius * hp.radius) {
                            result.push_back(idx);
                        }
                    }
                }
            }
        }
    }
};
