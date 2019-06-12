//
// Created by Xinming Zhang on 5/13/19.
//

#include "Integrator.h"

namespace FRACTURE{
    Eigen::Matrix3d mf(glm::vec3& force){
        Eigen::Matrix3d mf;
        float force_norm = glm::length(force);
        if(force_norm == 0){
            mf << 0, 0, 0,
                  0, 0, 0,
                  0, 0, 0;
            return mf;
        }
        double a = force.x;
        double b = force.y;
        double c = force.z;
        mf << a*a, a*b, a*c,
              b*a, b*b, b*c,
              c*a, c*b, c*c;
        return mf / force_norm;
    }
}

void ForwardEulerIntegrator::Integrate(MassSpringSystem *system, float timestep){
    // Get positions & velocities
    int numPoint = system->GetNumPoint();
    std::vector<glm::vec3> pos , vel;
    system->GetPositions(pos);
    system->GetVelocities(vel);

    // Compute accelerations
    std::vector<glm::vec3> acc;
    acc.clear();
    std::vector<std::vector<glm::vec3>> tensile_forces(numPoint, std::vector<glm::vec3>());
    std::vector<std::vector<glm::vec3>> compress_forces(numPoint, std::vector<glm::vec3>());
    std::vector<Eigen::Matrix3d> sep_tensors(numPoint);

    system->ComputeAccelerations(acc, tensile_forces, compress_forces);

    // Forward Euler step
    for(int i=0; i<numPoint; i++) {
        vel[i] += timestep * acc[i];
        pos[i] += vel[i] * timestep;

    }
    // Store results system->SetPositions(pos); system->SetVelocities(vel);
    system->SetPositions(pos);
    system->SetVelocities(vel);
    if(system->fracture){
        seperation_tensor_calculation(tensile_forces, compress_forces, sep_tensors);
        std::tuple<int, int, double, Eigen::Vector3cd> sep_point = speration_point_calculation(sep_tensors);
        if(std::get<2>(sep_point) > system->roughness){
            // fracture happened
            system->system_fracture(sep_point);
        }

    }

}

void ForwardEulerIntegrator::seperation_tensor_calculation(std::vector<std::vector<glm::vec3>> &tensile_forces,
                                              std::vector<std::vector<glm::vec3>> &compress_forces,
                                              std::vector<Eigen::Matrix3d>& sep_tensors) {
    int numPoint = sep_tensors.size();
    for (int i = 0; i < numPoint; ++i) {
        glm::vec3 f_plus(0);
        glm::vec3 f_minus(0);
        Eigen::Matrix3d sp;
        sp.setZero();
        for (int j = 0; j < tensile_forces[i].size(); ++j) {
            f_plus += tensile_forces[i][j];
            sp += FRACTURE::mf(tensile_forces[i][j]);
        }
        for (int k = 0; k < compress_forces[i].size(); ++k) {
            f_minus += compress_forces[i][k];
            sp -= FRACTURE::mf(compress_forces[i][k]);
        }
        sp -= FRACTURE::mf(f_plus);
        sp += FRACTURE::mf(f_minus);
        sep_tensors[i] = sp;
    }
}

std::tuple<int, int, double, Eigen::Vector3cd> ForwardEulerIntegrator::speration_point_calculation(std::vector<Eigen::Matrix3d> &sep_tensors) {
    int numPoint = sep_tensors.size();
    // point and eigen value index
    int point = 0;
    int eigen_index = -1;
    double eigen_value = 0;
    Eigen::Vector3cd eigen_vector;
    for (int i = 0; i < numPoint; ++i) {
//        std::cout << "---------" << i << "-----------" << "\n";
//        std::cout << sep_tensors[i] << "\n";
        Eigen::EigenSolver<Eigen::Matrix3d> es(sep_tensors[i]);
        for(int k = 0; k < 3; ++k){
            float eigen = float(es.eigenvalues()[k].real());
            if (eigen > eigen_value){
                point = i;
                eigen_index = k;
                eigen_value = eigen;
                eigen_vector = es.eigenvectors().col(k);
            }
        }
    }
    return std::make_tuple(point, eigen_index, eigen_value, eigen_vector);

}

/*
void MidpointIntegrator::Integrate(PhysicsSystem *system, float timestep) {
    // Get positions & velocities
    int numPoint = system->GetNumPoint();
    std::vector<glm::vec3> pos, pos_tmp, vel, vel_tmp(numPoint);
    system->GetPositions(pos);
    system->GetPositions(pos_tmp);
    system->GetVelocities(vel_tmp);
    system->GetVelocities(vel);

    // Compute accelerations
    std::vector<glm::vec3> acc_full;
    std::vector<glm::vec3> acc_half;
    acc_full.clear();
    acc_half.clear();
    system->ComputeAccelerations(acc_full);

    for(int i=0; i<numPoint; i++) {
//        std::cout << i+1 << ":" << acc[i].x << " " << acc[i].y << " " << acc[i].z <<"\n";
        vel_tmp[i] += acc_full[i] * timestep * 0.5f;
        vel_tmp[i].x = (fabsf(vel_tmp[i].x) < EPIS)? 0 : vel_tmp[i].x;
        vel_tmp[i].y = (fabsf(vel_tmp[i].y) < EPIS)? 0 : vel_tmp[i].y;
        vel_tmp[i].z = (fabsf(vel_tmp[i].z) < EPIS)? 0 : vel_tmp[i].z;

        pos_tmp[i] += acc_full[i] * timestep * 0.5f;

        pos_tmp[i].x = (fabsf(pos_tmp[i].x) < EPIS)? 0 : pos_tmp[i].x;
        pos_tmp[i].y = (fabsf(pos_tmp[i].y) < EPIS)? 0 : pos_tmp[i].y;
        pos_tmp[i].z = (fabsf(pos_tmp[i].z) < EPIS)? 0 : pos_tmp[i].z;

    }
    // Store results system->SetPositions(pos); system->SetVelocities(vel);
    system->SetPositions(pos_tmp);
    system->SetVelocities(vel_tmp);

    pos_tmp.clear();
    vel_tmp.clear();

    system->ComputeAccelerations(acc_half);
    for(int i=0; i<numPoint; i++) {
        vel[i] += acc_half[i] * timestep;
        vel[i].x = (fabsf(vel[i].x) < EPIS)? 0 : vel[i].x;
        vel[i].y = (fabsf(vel[i].y) < EPIS)? 0 : vel[i].y;
        vel[i].z = (fabsf(vel[i].z) < EPIS)? 0 : vel[i].z;

//        std::cout << i+1 << ":" << vel[i].x << " " << vel[i].y << " " << vel[i].z <<"\n";

        pos[i] += vel[i] * timestep;

        pos[i].x = (fabsf(pos[i].x) < EPIS)? 0 : pos[i].x;
        pos[i].y = (fabsf(pos[i].y) < EPIS)? 0 : pos[i].y;
        pos[i].z = (fabsf(pos[i].z) < EPIS)? 0 : pos[i].z;
    }
    system->SetPositions(pos);
    system->SetVelocities(vel);
}
 */