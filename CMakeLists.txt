cmake_minimum_required(VERSION 3.12)
project(PhysicsSimulation)

set(CMAKE_CXX_STANDARD 14)

add_executable(PhysicsSimulation main.cpp
        camera.h
        plane.h plane.cpp
        shader.h
        sphere.h
        mesh.h
        Integrator.h
        physys.h physys.cpp
        Integrator.cpp
        object_collision.h
        debugger.h
        debugger.cpp
        ./glm-aabb/AABB.hpp
        ./glm-aabb/AABB.cpp)
#tetra_intersect.h)

target_link_libraries(PhysicsSimulation libglfw.3.2.dylib
                                        libglad.a)