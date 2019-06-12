//
// Created by Xinming Zhang on 5/19/19.
//
#include <iostream>
#include <limits>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include "camera.h"
#include "shader.h"
#include "physys.h"
#include "mesh.h"
#include "plane.h"
#include "Integrator.h"
#include "debugger.h"
#include "object_collision.h"
#include "tetra_intersect.h"
#include "tetra_intersect_volume.h"
#include <set>
#include "Qhull.h"
#include "libqhullcpp/PointCoordinates.h"
#include "eigen3/Eigen/Dense"


int main(){
    std::vector<glm::vec3> tetra1 = {
            glm::vec3(0,1,0),
            glm::vec3(1,0,0),
            glm::vec3(0,0,1),
            glm::vec3(0,0,0)
    };

    std::vector<glm::vec3> tetra2 = {
            glm::vec3(0,1.5,0),
            glm::vec3(1,0.5,0),
            glm::vec3(0,0.5,1),
            glm::vec3(0,0.5,0)
    };
    std::vector<glm::vec3> tetra3 = {
            glm::vec3(1.1, 0.5, -0.5),
            glm::vec3(1.5, 1.5, 0),
            glm::vec3(1.1, 0.5, 0.5),
            glm::vec3(1.5, 0.5, 0)
    };
//    std::cout << TETRA_INTER::tet_a_tet(tetra1, tetra3);
//    std::vector<glm::vec3> result = TETRA_VOLUME::intersection_test(tetra1, tetra3);
//    result.push_back(result[1]);
//    result.push_back(result[1]);

//    float volume = TETRA_VOLUME::intersection_volume(tetra3);
//    std::cout << volume << "\n";

    Eigen::Matrix3d m;
    m << 1,2,3,
         2,3,4,
         3,4,5;
    Eigen::EigenSolver<Eigen::Matrix3d> es(m);
    Eigen::Vector3cd evc = es.eigenvectors().col(0);
    std::cout << evc << "\n";


}
