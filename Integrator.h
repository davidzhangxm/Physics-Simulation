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
    virtual void Integrate(MassSpringSystem *system, float timestep){};
};

class ForwardEulerIntegrator: public Integrator{
public:

    ForwardEulerIntegrator() = default;
    ~ForwardEulerIntegrator() = default;

    void Integrate(MassSpringSystem *sys, float timestep) override;
    void seperation_tensor_calculation(std::vector<std::vector<glm::vec3>>& tensile_forces,
                          std::vector<std::vector<glm::vec3>>& compress_forces,
                          std::vector<Eigen::Matrix3d>& sep_tensors);
    std::tuple<int, int, double, Eigen::Vector3cd> speration_point_calculation(std::vector<Eigen::Matrix3d>& sep_tensors);
};


class MidpointIntegrator: public Integrator{
public:
    MidpointIntegrator() = default;
    ~MidpointIntegrator() = default;

    void Integrate(MassSpringSystem *sys, float timestep) override;
};

#endif //PHYSICSSIMULATION_INTEGRATOR_H
