//
// Created by Xinming Zhang on 5/14/19.
//
#pragma once
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
#include "glm-aabb/AABB_tree.h"

#include "tetra_intersect_volume.h"
#include "tetra_intersect.h"

namespace CollideQuery {
    // linear parameter in terms of intersection colume
    // to calcualte the force of tetra collision force
    float collision_force_parameter = 5;

    // leaf node to leaf node
    void query(unsigned tree_node_1_index, unsigned tree_node_2_index, std::vector<AABBnode> &tree_node_1, std::vector<AABBnode> &tree_node_2, std::vector<std::pair<unsigned int, unsigned int>> &tetra_collide_result)
    {
        if(!tree_node_1[tree_node_1_index].aabb.overlaps(tree_node_2[tree_node_2_index].aabb))
            return;
        if(tree_node_1[tree_node_1_index].isLeaf() && tree_node_2[tree_node_2_index].isLeaf())
        {
            tetra_collide_result.push_back(std::make_pair(tree_node_1[tree_node_1_index].tetra.ID, tree_node_2[tree_node_2_index].tetra.ID));
            return;
        }
        else if(tree_node_1[tree_node_1_index].isLeaf() && !tree_node_2[tree_node_2_index].isLeaf() ){
            query(tree_node_1_index, tree_node_2[tree_node_2_index].leftNodeIndex, tree_node_1,tree_node_2, tetra_collide_result);
            query(tree_node_1_index, tree_node_2[tree_node_2_index].rightNodeIndex, tree_node_1,tree_node_2, tetra_collide_result);
        }
        else if(!tree_node_1[tree_node_1_index].isLeaf() && tree_node_2[tree_node_2_index].isLeaf()){
            query(tree_node_1[tree_node_1_index].leftNodeIndex, tree_node_2_index, tree_node_1, tree_node_2, tetra_collide_result);
            query(tree_node_1[tree_node_1_index].rightNodeIndex, tree_node_2_index, tree_node_1, tree_node_2, tetra_collide_result);
        }
        else{
            query(tree_node_1[tree_node_1_index].leftNodeIndex, tree_node_2[tree_node_2_index].leftNodeIndex, tree_node_1, tree_node_2, tetra_collide_result);
            query(tree_node_1[tree_node_1_index].rightNodeIndex, tree_node_2[tree_node_2_index].leftNodeIndex, tree_node_1, tree_node_2, tetra_collide_result);
            query(tree_node_1[tree_node_1_index].leftNodeIndex, tree_node_2[tree_node_2_index].rightNodeIndex, tree_node_1, tree_node_2, tetra_collide_result);
            query(tree_node_1[tree_node_1_index].rightNodeIndex, tree_node_2[tree_node_2_index].rightNodeIndex, tree_node_1, tree_node_2, tetra_collide_result);
        }
    }
    // abb tree collide detection
    std::vector<std::pair<unsigned int, unsigned int>> object_collide(AABB_Tree& aabb_tree1, AABB_Tree& aabb_tree2)
    {
        std::vector<std::pair<unsigned int, unsigned int>> tetra_collide_result;
        unsigned tree_root_1_index = aabb_tree1._rootNodeIndex;
        unsigned tree_root_2_index = aabb_tree2._rootNodeIndex;
        std::vector<AABBnode> tree_node_1 = aabb_tree1.get_node_list();
        std::vector<AABBnode> tree_node_2 = aabb_tree2.get_node_list();
        query(tree_root_1_index, tree_root_2_index, tree_node_1, tree_node_2, tetra_collide_result);

        return tetra_collide_result;
    }

    // first step to detect two objects collision
    std::vector<std::tuple<int, int, std::vector<std::pair<unsigned int, unsigned int>>>> collide_quert_list(std::vector<MassSpringSystem>& list){
        std::vector<std::tuple<int, int, std::vector<std::pair<unsigned int, unsigned int>>>> collide_result;
        int size = list.size();
        for(int i = 0; i < size-1; ++i){
            for(int j = i+1; j < size; ++j){
                AABB_Tree aabb_tree1 = list[i].get_aabb_tree();
                AABB_Tree aabb_tree2 = list[j].get_aabb_tree();

                std::vector<std::pair<unsigned int, unsigned int>> tetra_collide_list = object_collide(aabb_tree1, aabb_tree2);
                if(!tetra_collide_list.empty())
                    collide_result.push_back(std::make_tuple(i, j, tetra_collide_list));
            }
        }
        return collide_result;
    }
    // when two tetrahedron in two objects collide each other
    // find the intersection polyhedron volume
    // calculate force response and
    // apply back to objects
    void collision_response(std::vector< std::tuple< int, int, std::vector<std::pair<unsigned int, unsigned int>> > >& collide_quert_list, std::vector<MassSpringSystem>& list){
        int N = (int)collide_quert_list.size();
        for(int n = 0; n < N; ++n){
            int objetc1 = std::get<0>(collide_quert_list[n]);
            int objetc2 = std::get<1>(collide_quert_list[n]);

            // tetra to tetra collide list to verify
            std::vector<std::pair<unsigned int, unsigned int>> collide_list = std::get<2>(collide_quert_list[n]);
            int n_tetra_tetra = collide_list.size();
            for(int i = 0; i < n_tetra_tetra; ++i){
                unsigned int tetra1_id = collide_list[i].first;
                unsigned int tetra2_id = collide_list[i].second;
                std::vector<glm::vec3> tetra1;
                std::vector<glm::vec3> tetra2;
                list[objetc1].get_tetra(tetra1_id, tetra1);
                list[objetc2].get_tetra(tetra2_id, tetra2);

                // tetra collidsion detection
                bool collide = TETRA_INTER::tet_a_tet(tetra1, tetra2);
                if(collide){
//                    std::vector<glm::vec3> polyhedron = TETRA_VOLUME::intersection_test(tetra1, tetra2);

                    glm::vec3 tetra1_mass_center = TETRA_VOLUME::mass_center(tetra1);
                    glm::vec3 tetra2_mass_center = TETRA_VOLUME::mass_center(tetra2);

                    //mass center for tetra2 and
                    // collision force on tetra2
                    glm::vec3 tetra1_force = glm::normalize(tetra1_mass_center - tetra2_mass_center) * collision_force_parameter;
                    glm::vec3 tetra2_force = glm::normalize(tetra2_mass_center - tetra1_mass_center) * collision_force_parameter;

                    // apply force on four points of tetrahedron
                    list[objetc1].set_collision_force(tetra1_id, tetra1_force);
                    list[objetc2].set_collision_force(tetra2_id, tetra2_force);

//                    // we have a intersection polyhedron
//                    float intersection_volume = TETRA_VOLUME::intersection_volume(polyhedron);
//                    glm::vec3 mass_center = TETRA_VOLUME::mass_center(polyhedron);
//                    float total_force = intersection_volume * collision_force_parameter;
//                    // mass center for tetra1 and
//                    // collision force on tetra1
//                    glm::vec3 tetra1_mass_center = TETRA_VOLUME::mass_center(tetra1);
//                    glm::vec3 tetra2_mass_center = TETRA_VOLUME::mass_center(tetra2);
//
//
//                    // mass center for tetra2 and
//                    // collision force on tetra2
//                    glm::vec3 tetra1_force = glm::normalize(tetra1_mass_center - tetra2_mass_center) * total_force;
//                    glm::vec3 tetra2_force = glm::normalize(tetra2_mass_center - tetra1_mass_center) * total_force;
//
//                    // apply force on four points of tetrahedron
//                    list[objetc1].set_collision_force(tetra1_id, mass_center, tetra1_force);
//                    list[objetc2].set_collision_force(tetra2_id, mass_center, tetra2_force);
                }

            }

        }
    }

    void collision_response_simple(std::vector< std::tuple< int, int, std::vector<std::pair<unsigned int, unsigned int>> > >& collide_quert_list, std::vector<MassSpringSystem>& list){
        int N = (int)collide_quert_list.size();
        for(int n = 0; n < N; ++n){
            int objetc1 = std::get<0>(collide_quert_list[n]);
            int objetc2 = std::get<1>(collide_quert_list[n]);

            // tetra to tetra collide list to verify
            std::vector<std::pair<unsigned int, unsigned int>> collide_list = std::get<2>(collide_quert_list[n]);
            int n_tetra_tetra = collide_list.size();
            for(int i = 0; i < n_tetra_tetra; ++i){
                unsigned int tetra1_id = collide_list[i].first;
                unsigned int tetra2_id = collide_list[i].second;
                std::vector<glm::vec3> tetra1;
                std::vector<glm::vec3> tetra2;
                list[objetc1].get_tetra(tetra1_id, tetra1);
                list[objetc2].get_tetra(tetra2_id, tetra2);


            }

        }

    }


    // clear object collision response
    void clear_collision_response(std::vector<MassSpringSystem>& list){
        int n = list.size();
        for (int i = 0; i < n; ++i) {
            list[i].collision_force_clear();
        }
    }

}

#endif //PHYSICSSIMULATION_OBJECT_COLLISION_H
