//
// Created by Xinming Zhang on 4/19/19.
//

#ifndef PHYSICSSIMULATION_PHYSYS_H
#define PHYSICSSIMULATION_PHYSYS_H



#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include "shader.h"


class PhysicsSystem{
public:
//    virtual int GetNumDofs();
    virtual void GetPositions(std::vector<glm::vec3> &pos){}
    virtual void GetVelocities(std::vector<glm::vec3> &vel){}
    virtual unsigned int GetNumPoint(){return 0;}

    virtual void SetPositions(const std::vector<glm::vec3> &pos){}
    virtual void SetVelocities(const std::vector<glm::vec3> &vel){}
    virtual void ComputeAccelerations(std::vector<glm::vec3> &acc){}

    virtual std::vector<glm::vec3> getVertex() {}
    virtual std::vector<unsigned int> getIndex() {}
    virtual std::vector<glm::vec3> getVelocity() {}
    virtual std::vector<glm::vec3> getAcceleration(){};
    virtual std::vector<float> getMass(){}
};

class MassSpringSystem: public PhysicsSystem{
public:
    MassSpringSystem(const char* vertexfile,
                     const char* edgefile,
                     const char* tetrahedrafile,
                     float densities,
                     glm::vec3 location,
                     float v,
                     float E,
                     float v_damping,
                     float E_damping,
                     float timestep,
                     bool use_gravity);
    ~MassSpringSystem() = default;

    void setMesh(const char* vertexfile, const char* facefile, const char* tetrahedrafile);
    void readVertex(const char* vertexfile);
    void readFacefile(const char* facefile);
    void readTetrafile(const char* tetrahedrafile);
    void readIndex(std::vector<unsigned int> tetrahedra);
    void addVertex(float x, float y, float z);
    void addFace(unsigned int a, unsigned int b, unsigned int c);
    void addTetra(unsigned int a, unsigned int b, unsigned int c, unsigned int d);
    void addIndex(unsigned int a, unsigned int b, unsigned int c);
    void massRCalculation();
    void velocityInitialization();

    void GetPositions(std::vector<glm::vec3> &pos);
    void GetVelocities(std::vector<glm::vec3> &vel);

    unsigned int GetNumPoint();

    void SetPositions(const std::vector<glm::vec3> &pos);
    void SetVelocities(const std::vector<glm::vec3> &vel);
    void ComputeAccelerations(std::vector<glm::vec3> &acc);


    std::vector<glm::vec3> getVertex();
    std::vector<unsigned int> getIndex();
    std::vector<glm::vec3> getVelocity();
    std::vector<glm::vec3> getAcceleration();
    std::vector<float> getMass();
    // initialize shader
    void initShader();
    // set transformation parametre
    void set_transformation(glm::mat4 transform);
    void render_system();
    void delete_shader();

private:
    // geometry parameter
    unsigned int numPoint;
    unsigned int numTetre;
    std::vector<glm::vec3> vertex;
    std::vector<unsigned int> index;
    std::vector<unsigned int> face;
    std::vector<unsigned int> tetrahedra;

    // integration parameter
    float dens;
    float timestep;
    bool use_gravity;
    glm::vec3 gravity;
    glm::vec3 location;
    std::vector<float> mass;
    float lambda;
    float lambda_damping;
    float miu;
    float miu_damping;
    std::vector<glm::vec3> velocities;
    std::vector<glm::vec3> accelerations;
    std::vector<glm::mat3> RestFrame;
    std::vector<glm::mat3> preEps;

    // shader parameter
    unsigned int vao;
    unsigned int vbo;
    unsigned int ebo;
    Shader m_system_shader;
    glm::mat4 transform;
};


#endif //PHYSICSSIMULATION_PHYSYS_H
