//
// Created by Xinming Zhang on 4/19/19.
//

#ifndef PHYSICSSIMULATION_INTEGRATOR_H
#define PHYSICSSIMULATION_INTEGRATOR_H

#include "physys.h"

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

        vel[i].x += acc[i].x * timestep;
        vel[i].y += acc[i].y * timestep;
        vel[i].z += acc[i].z * timestep;

        pos[i].x += vel[i].x * timestep;
        pos[i].y += vel[i].y * timestep;
        pos[i].z += vel[i].z * timestep;

    }
    // Store results system->SetPositions(pos); system->SetVelocities(vel);
    system->SetPositions(pos);
    system->SetVelocities(vel);
}

#endif //PHYSICSSIMULATION_INTEGRATOR_H
