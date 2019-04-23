//
// Created by Xinming Zhang on 4/19/19.
//

#ifndef PHYSICSSIMULATION_INTEGRATOR_H
#define PHYSICSSIMULATION_INTEGRATOR_H

#include "physys.h"

#define EPIS 1e-5

class Integrator{
public:
    Integrator() = default;
    ~Integrator() = default;
    virtual void Integrate(PhysicsSystem *system, float timestep){};
};

class ForwardEulerIntegrator: public Integrator{
public:

    ForwardEulerIntegrator() = default;
    ~ForwardEulerIntegrator() = default;

    void Integrate(PhysicsSystem *sys, float timestep) override;
};

void ForwardEulerIntegrator::Integrate(PhysicsSystem *system, float timestep){
    // Get positions & velocities
    int numPoint = system->GetNumPoint();
    std::vector<glm::vec3> pos , vel;
    system->GetPositions(pos);
    system->GetVelocities(vel);

    // Compute accelerations
    std::vector<glm::vec3> acc;
    acc.clear();
    system->ComputeAccelerations(acc);


    // Forward Euler step
    for(int i=0; i<numPoint; i++) {
        std::cout << i+1 << ":" << acc[i].x << " " << acc[i].y << " " << acc[i].z <<"\n";
        vel[i] += timestep * acc[i];
        pos[i] += vel[i] * timestep;

    }
    // Store results system->SetPositions(pos); system->SetVelocities(vel);
    system->SetPositions(pos);
    system->SetVelocities(vel);
}

class MidpointIntegrator: public Integrator{
public:
    MidpointIntegrator() = default;
    ~MidpointIntegrator() = default;

    void Integrate(PhysicsSystem *sys, float timestep) override;
};

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

        std::cout << i+1 << ":" << vel[i].x << " " << vel[i].y << " " << vel[i].z <<"\n";

        pos[i] += vel[i] * timestep;

        pos[i].x = (fabsf(pos[i].x) < EPIS)? 0 : pos[i].x;
        pos[i].y = (fabsf(pos[i].y) < EPIS)? 0 : pos[i].y;
        pos[i].z = (fabsf(pos[i].z) < EPIS)? 0 : pos[i].z;
    }
    system->SetPositions(pos);
    system->SetVelocities(vel);
}


#endif //PHYSICSSIMULATION_INTEGRATOR_H
