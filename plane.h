//
// Created by Xinming Zhang on 4/15/19.
//
#pragma once

#ifndef PHYSICSSIMULATION_OBJECT_H
#define PHYSICSSIMULATION_OBJECT_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "physys.h"
#include "shader.h"

#define VERTEX_SHADER ("shader/ground.vs")
#define FRAGRANT_SHADER ("shader/ground.fs")

class PhysicsSystem;

class Plane{
public:
    // creator
    Plane(glm::vec3 origin,
          glm::vec3 side1,
          glm::vec3 side2,
          float distance);
    ~Plane() = default;

    // manipulateor
    struct Coef{
        float m_reflection;
        float m_friction;
        float m_repulsion;
        Coef();
    } m_corf;
    void processCollision(PhysicsSystem& phy);
    void initShaders();
    void renderPlane();
    void setTransform(glm::mat4 transform);
    void deleteBuffer();

    // accessor

private:
    Shader plane_shader;
    unsigned int vao;
    unsigned int vbo;
    unsigned int ebo;
    glm::vec3 m_normal;
    glm::vec3 m_origin;
    glm::vec3 m_point1;
    glm::vec3 m_point2;
    glm::vec3 m_point3;
    std::vector<glm::vec3> point;
    std::vector<unsigned int> index;
    glm::mat4 transform;

};

#endif //PHYSICSSIMULATION_OBJECT_H
