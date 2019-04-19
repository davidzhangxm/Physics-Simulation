//
// Created by Xinming Zhang on 4/15/19.
//

#include "object.h"

Ground::Ground(glm::vec3 origin, glm::vec3 side1, glm::vec3 side2) {
    this->origin = origin;
    this->p2 = origin + side1;
    this->p3 = origin + side1 + side2;
    this->p4 = origin + side2;
    glm::vec3 nm = glm::cross(side1, side2);
    this->normal = nm / glm::length(nm);
}

glm::vec3 Ground::getNormal() {
    return this->normal;
}