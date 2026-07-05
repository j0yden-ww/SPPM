cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

# 编译优化：-O3 激进优化, -ffast-math 放宽浮点精度换速度, -march=native 启用本机 SIMD 指令
add_compile_options(-O3 -ffast-math -march=native)

add_executable(RayTracing
    main.cpp
    core/Vector.cpp
    object/BVH.cpp
    scene/Scene.cpp
    renderer/Renderer.cpp
)

target_include_directories(RayTracing PRIVATE
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/object
    ${CMAKE_SOURCE_DIR}/scene
    ${CMAKE_SOURCE_DIR}/renderer
)
