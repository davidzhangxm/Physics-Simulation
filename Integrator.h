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


class MidpointIntegrator: public Integrator{
public:
    MidpointIntegrator() = default;
    ~MidpointIntegrator() = default;

    void Integrate(PhysicsSystem *sys, float timestep) override;
};

#endif //PHYSICSSIMULATION_INTEGRATOR_H
