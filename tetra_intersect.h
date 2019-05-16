//
// Created by Xinming Zhang on 5/15/19.
//

#ifndef PHYSICSSIMULATION_TETRA_INTERSECT_H
#define PHYSICSSIMULATION_TETRA_INTERSECT_H

#include <iostream>
#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <glm/gtx/normal.hpp>

namespace TETRA_INTER {

// counter clock wise for three points
glm::vec3 calculate_normal(glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &p3){
    glm::vec3 n1 = p3 - p1;
    glm::vec3 n2 = p2 - p1;
    glm::vec3 normal = glm::normalize(glm::cross(n1, n2));
    return normal;
}

int tet_to_plane(std::vector<glm::vec3> &plane, std::vector<glm::vec3> &tet){
    glm::vec3 n = calculate_normal(plane[0], plane[1], plane[2]);
    int mask = 0;
    for (int i = 0; i < 4; ++i) {
        glm::vec3 tp = tet[i] - plane[0];
        if(glm::dot(n, tp) > 0)
            mask |= (1 << i);
    }
    return mask;
}

bool tetA_to_tetB(std::vector<glm::vec3> &tet1, std::vector<glm::vec3> &tet2){
    int mask[4];
    std::vector<glm::vec3> plane1 = {tet1[0], tet1[1], tet1[2]};
    std::vector<glm::vec3> plane2 = {tet1[0], tet1[3], tet1[1]};
    std::vector<glm::vec3> plane3 = {tet1[0], tet1[2], tet1[3]};
    std::vector<glm::vec3> plane4 = {tet1[1], tet1[3], tet1[2]};
    mask[0] = tet_to_plane(plane1, tet2);
    mask[1] = tet_to_plane(plane2, tet2);
    mask[2] = tet_to_plane(plane3, tet2);
    mask[3] = tet_to_plane(plane4, tet2);
    if((mask[0] | mask[1] | mask[2] | mask[3]) == 15)
        return false;
    return true;
}

bool tet_to_tet(std::vector<glm::vec3> &tet1, std::vector<glm::vec3> &tet2){
    bool t1t2 = tetA_to_tetB(tet1, tet2);
    bool t2t1 = tetA_to_tetB(tet2, tet1);
    return t1t2 | t2t1;
}

}

#endif //PHYSICSSIMULATION_TETRA_INTERSECT_H
