//
// Created by Xinming Zhang on 4/15/19.
//

#include "plane.h"

Plane::Coef::Coef()
: m_reflection(0.6f)
, m_friction(0.1f)
, m_repulsion(2.0f)
{}

Plane::Plane(glm::vec3 origin,
             glm::vec3 side1,
             glm::vec3 side2,
             float distance) {
    this->m_origin = origin;
    this->m_point1 = side1 * distance + origin;
    this->m_point2 = side2 * distance + origin;
    this->m_point3 = origin + side1 * distance + side2 * distance;
    this->m_normal = glm::normalize(glm::cross(side2,side1));
    point.push_back(m_origin);
    point.push_back(m_point1);
    point.push_back(m_point3);
    point.push_back(m_point2);
    index = {
            0, 1, 2,
            0, 2, 3
    };
}

void Plane::processCollision(PhysicsSystem &phy) {
    glm::vec3 normal = m_normal;
    int numPoint = phy.GetNumPoint();
    for(int i = 0; i < numPoint; ++i)
    {

        glm::vec3 p_position = phy.getVertex()[i];
        glm::vec3 p_velovity = phy.getVelocity()[i];

        float dx = glm::dot(normal, p_position - m_origin);
        if(dx >= 0)
            continue;
        phy.getVertex()[i] -= dx * normal;

        float dv = glm::dot(normal, p_velovity);
        glm::vec3 v_normal = dv * normal;
        phy.getVelocity()[i] -= m_corf.m_reflection* v_normal;
        phy.getVelocity()[i] *= (1.0f - m_corf.m_friction);

        phy.getAcceleration()[i] += m_corf.m_repulsion * normal / phy.getMass()[i];
    }
}

void Plane::initShaders() {
    plane_shader = Shader(VERTEX_SHADER, FRAGRANT_SHADER);

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, 4*sizeof(glm::vec3), &point[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6*sizeof(unsigned int), &index[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Plane::renderPlane() {
    plane_shader.use();
    plane_shader.setMat4("transform", transform);
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    plane_shader.unuse();
}

void Plane::setTransform(glm::mat4 transform) {
    this->transform = transform;
}

void Plane::deleteBuffer() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
}