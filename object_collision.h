//
// Created by Xinming Zhang on 5/14/19.
//

#ifndef PHYSICSSIMULATION_OBJECT_COLLISION_H
#define PHYSICSSIMULATION_OBJECT_COLLISION_H

#include <iostream>
#include <vector>
#include <utility>


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "physys.h"
#include "glm-aabb/AABB.hpp"

class CollisionSolver{
public:
    CollisionSolver(std::vector<MassSpringSystem>& system_list);
    ~CollisionSolver() = default;

    void CollisionSolve();
    void collision_detection();
    void collision_response();
private:
    std::vector<MassSpringSystem> list;
    std::vector<std::pair<int, unsigned int>> collide;
    int size;

    bool object_overlap(MassSpringSystem& s1, MassSpringSystem &s2);
    bool tetra_overlap();
};

// intialization
CollisionSolver::CollisionSolver(std::vector<MassSpringSystem>& system_list) {
    this->list.assign(system_list.begin(), system_list.end());
    size = system_list.size();
}

void CollisionSolver::CollisionSolve() {
    collision_detection();
}

void CollisionSolver::collision_detection() {
    for(int i = 0; i < size-1; ++i)
        for(int j = 1; j < size; ++j){
            if(object_overlap(list[i], list[j])){
                collide.push_back(std::make_pair<i, tetra1>);
                collide.push_back(std::make_pair<j, tetra2>);
            }
        }
}

bool CollisionSolver::object_overlap(MassSpringSystem &s1, MassSpringSystem &s2) {
    CPM_GLM_AABB_NS::AABB s1_aabb = s1.get_aabb();
    CPM_GLM_AABB_NS::AABB s2_aabb = s2.get_aabb();
    return s1_aabb.overlaps(s2_aabb);
}

#endif //PHYSICSSIMULATION_OBJECT_COLLISION_H
