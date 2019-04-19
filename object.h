//
// Created by Xinming Zhang on 4/15/19.
//

#ifndef PHYSICSSIMULATION_OBJECT_H
#define PHYSICSSIMULATION_OBJECT_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Ground{
public:
    // creator
    Ground(glm::vec3 origin, glm::vec3 side1, glm::vec3 side2);
    ~Ground();

    // manipulateor

    // accessor
    glm::vec3 getNormal();
    unsigned int* getIndices();
    float* getVertices();

private:
    glm::vec3 origin;
    glm::vec3 p2;
    glm::vec3 p3;
    glm::vec3 p4;
    glm::vec3 normal;
    float vertices[4];
    unsigned int indices[6];
};

#endif //PHYSICSSIMULATION_OBJECT_H
