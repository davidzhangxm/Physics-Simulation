//
// Created by Xinming Zhang on 5/19/19.
//

#ifndef PHYSICSSIMULATION_TETRA_INTERSECT_VOLUME_H
#define PHYSICSSIMULATION_TETRA_INTERSECT_VOLUME_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <set>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Qhull.h"
#include "libqhullcpp/PointCoordinates.h"

namespace TETRA_VOLUME{
    struct Plane{
        glm::vec3 norm;
        glm::vec3 p1;
        Plane() = default;
        Plane(glm::vec3& p1, glm::vec3& p2, glm::vec3& p3){
            norm = glm::normalize(glm::cross(p3 - p1, p2 - p1));
            this->p1 = p1;
        }
    };
    void insert(std::vector<glm::vec3>& result_point, glm::vec3 &new_point){
        int n = (int)result_point.size();
        for (int i = 0; i < n; ++i) {
            glm::vec3 tmp = result_point[i];
            if((tmp.x == new_point.x) & (tmp.y == new_point.y) & (tmp.z == new_point.z))
                return;
        }
        result_point.push_back(new_point);
    }

    std::vector<glm::vec3> triangle_genration(std::vector<glm::vec3> points){
        int n = (int)points.size();
        int r = 3;

        assert( r <= n );

        std::vector<glm::vec3> result;
        std::vector<bool> v(n);
        std::fill(v.end()- r, v.end(), true);
        do{
            for (int i = 0; i < n; ++i) {
                if(v[i]) result.push_back(points[i]);
            }
        } while(std::next_permutation(v.begin(), v.end()));
        return result;
    }

    void clip(Plane& plane, std::vector<glm::vec3>& tetra_points){
        std::vector<glm::vec3> triangles = triangle_genration(tetra_points);
        int triangle_size = (int)triangles.size() / 3;
        std::vector<glm::vec3> new_points;

        for(int i = 0; i < triangle_size; ++i){
            for(int j = 0; j < 3; ++j){
                int k = (j+1) % 3;
                // position for first point
                float j_pos = glm::dot( (triangles[3*i + j] - plane.p1), plane.norm );
                // position for second point
                float k_pos = glm::dot( (triangles[3*i + k] - plane.p1), plane.norm );

                // Case 1: when both points are inside
                if(j_pos < 0 && k_pos < 0){
                    //only second point is added
                    insert(new_points, triangles[3*i + k]);
                }
                    // Case 2 : when only first point is outside
                else if(j_pos >= 0 && k_pos < 0){
                    // Point of intersection with edge
                    // and the second point is added
                    glm::vec3 intersection = (j_pos * triangles[3*i + k] - k_pos * triangles[3*i + j]) / (j_pos - k_pos);
                    insert(new_points, intersection);
                    insert(new_points, triangles[3*i + k]);
                }

                    // Case 3: when only second point is outside
                else if (j_pos < 0 && k_pos >= 0){
                    // only point of intersection with edge is added
                    glm::vec3 intersection = (j_pos * triangles[3*i + k] - k_pos * triangles[3*i + j]) / (j_pos - k_pos);
                    insert(new_points, intersection);
                }
                    // Case 4 when both points are outside
                else{
                    // no points are added
                }
            }

        }

        // coping new points into original array
        // and changing the # vertices
        tetra_points.clear();
        tetra_points.assign(new_points.begin(), new_points.end());

    }


    std::vector<glm::vec3> intersection_test(std::vector<glm::vec3>& tetra1, std::vector<glm::vec3>& tetra2){
        // Intersect 2 tetraheron objects and return the intersection volume
        Plane p1 = Plane(tetra1[0], tetra1[1], tetra1[2]);
        Plane p2 = Plane(tetra1[0], tetra1[2], tetra1[3]);
        Plane p3 = Plane(tetra1[0], tetra1[3], tetra1[1]);
        Plane p4 = Plane(tetra1[1], tetra1[3], tetra1[2]);
        std::vector<Plane> planes = {p1, p2, p3, p4};
        std::vector<glm::vec3> tetra_points(tetra2.begin(), tetra2.end());

        for (int i = 0; i < 4; ++i) {
            clip(planes[i], tetra_points);
            if(tetra_points.size() == 0)
                break;
        }
        return tetra_points;
    }

    float intersection_volume(const std::vector<glm::vec3>& tetra){
        double all_point[12];
        for (int i = 0; i < 4; ++i) {
            all_point[3*i+0] = tetra[i].x;
            all_point[3*i+1] = tetra[i].y;
            all_point[3*i+2] = tetra[i].z;

        }
        orgQhull::PointCoordinates *m_externalPoints;
        m_externalPoints = new orgQhull::PointCoordinates(3, "");
        orgQhull::Qhull qhull("", 3, 4, all_point, "Qt");

        float volume = qhull.volume();
        delete(m_externalPoints);
        return volume;
    }

    glm::vec3 mass_center(const std::vector<glm::vec3>& polyhedron){
        glm::vec3 m_center(0);
        for(int i = 0; i < polyhedron.size(); ++i){
            m_center += polyhedron[i];
        }
        return m_center / (float)polyhedron.size();
    }
}





#endif //PHYSICSSIMULATION_TETRA_INTERSECT_VOLUME_H
