#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// ===================================================================
// main — 场景构建与渲染入口
// ===================================================================
// 搭建 Cornell Box 场景，放置漫反射墙壁、镜子墙面、玻璃球、
// 面光源等物体，然后调用 SPPM 渲染器输出图像。
// ===================================================================

int main(int argc, char** argv)
{
    // 创建场景，500×500 分辨率
    Scene scene(500, 500);

    // ------------------- 材质定义 -------------------

    // 红色漫反射 — 用于 tallbox
    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.4f, 0.065f, 0.05f);

    // 绿色漫反射 — 用于右墙
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.30f, 0.091f);

    // 白色漫反射 — 用于地板
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);

    // 面光源材质 — 安装在模型 light.obj 上
    // m_emission 由多组色温数据加权混合，模拟真实白炽灯光谱
    Material* light = new Material(DIFFUSE,
        (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f)
       + 15.6f * Vector3f(0.740f+0.287f, 0.740f+0.160f, 0.740f)
       + 18.4f * Vector3f(0.737f+0.642f, 0.737f+0.159f, 0.737f)));
    light->Kd = Vector3f(0.6f);

    // 镜面反射材质 — 用于左墙
    Material* mirror = new Material(MIRROR, Vector3f(0.0f));
    mirror->Ks = Vector3f(0.9f, 0.9f, 0.9f);

    // 玻璃折射材质 — 用于玻璃球，IOR=1.5
    Material* glass = new Material(REFRACTIVE, Vector3f(0.0f));
    glass->Kd = Vector3f(0.0f);
    glass->Ks = Vector3f(0.95f, 0.95f, 0.95f);
    glass->ior = 1.5f;

    // ------------------- 场景物体 -------------------

    // 地板：白色漫反射
    MeshTriangle floor("models/cornellbox/floor.obj", white);
    // 高盒子：红色漫反射，位于右侧
    MeshTriangle tallbox("models/cornellbox/tallbox.obj", red);
    // 左墙：镜面反射
    MeshTriangle left("models/cornellbox/left.obj", mirror);
    // 右墙：绿色漫反射
    MeshTriangle right("models/cornellbox/right.obj", green);
    // 天花板光源
    MeshTriangle light_("models/cornellbox/light.obj", light);
    // 玻璃球：替换原 shortbox 位置，中心 (186, 85, 169)，半径 80
    Sphere glassSphere(Vector3f(186, 85, 169), 80.0f, glass);

    // 将物体加入场景
    scene.Add(&floor);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    scene.Add(&glassSphere);

    // 构建 BVH 加速结构
    scene.buildBVH();

    // ------------------- 渲染 -------------------

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: "
              << std::chrono::duration_cast<std::chrono::hours>(stop - start).count()
              << " hours\n";
    std::cout << "          : "
              << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count()
              << " minutes\n";
    std::cout << "          : "
              << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count()
              << " seconds\n";

    return 0;
}
