//
// Created by Xinming Zhang on 4/19/19.
//

#ifndef PHYSICSSIMULATION_PHYSYS_H
#define PHYSICSSIMULATION_PHYSYS_H



#include <iostream>
#include <vector>
#include <set>
#include <sstream>
#include <fstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include "shader.h"
#include "glm-aabb/AABB.hpp"
#include "glm-aabb/AABB_tree.h"

#include "eigen3/Eigen/Dense"


class PhysicsSystem{
public:


//    virtual int GetNumDofs();
    virtual void GetPositions(std::vector<glm::vec3> &pos){}
    virtual void GetVelocities(std::vector<glm::vec3> &vel){}
    virtual unsigned int GetNumPoint(){return 0;}
    virtual unsigned int GetNumTetra() {return 0;}

    virtual void SetPositions(const std::vector<glm::vec3> &pos){}
    virtual void SetVelocities(const std::vector<glm::vec3> &vel){}
    virtual void SetAccelerations(const std::vector<glm::vec3> &vec){}
    virtual void ComputeAccelerations(std::vector<glm::vec3> &acc,
                                      std::vector<std::vector<glm::vec3>>& tensile_forces,
                                      std::vector<std::vector<glm::vec3>>& compress_forces){}

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
                     bool use_gravity,
                     bool fracture,
                     float roughtness,
                     glm::vec3 lightPos,
                     glm::vec3 viewPos);
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
    void normalCalculation();
    void surfaceNormalCalculation();
    void parameterInitialization();

    void GetPositions(std::vector<glm::vec3> &pos);
    void GetVelocities(std::vector<glm::vec3> &vel);

    unsigned int GetNumPoint();
    unsigned int GetNumTetra();


    void SetPositions(const std::vector<glm::vec3> &pos);
    void SetVelocities(const std::vector<glm::vec3> &vel);
    void SetAccelerations(const std::vector<glm::vec3> &acc);
    void ComputeAccelerations(std::vector<glm::vec3> &acc,
                              std::vector<std::vector<glm::vec3>>& tensile_forces,
                              std::vector<std::vector<glm::vec3>>& compress_forces);


    std::vector<glm::vec3> getVertex();
    std::vector<unsigned int> getIndex();
    std::vector<unsigned int> getTetra();
    void get_tetra(unsigned int tetra_id, std::vector<glm::vec3>& tetra);
    std::vector<glm::vec3> getVelocity();
    std::vector<glm::vec3> getAcceleration();
    std::vector<float> getMass();
    // initialize shader
    void initShader();
    // set transformation parametre
    void set_transformation(glm::mat4 transform);
    void set_viewpos(glm::vec3 viewPos);
    void render_system();
    void delete_shader();
    void update();
    void collision_force_clear();
    void set_collision_force(unsigned int tetra_id, glm::vec3& force);

    // optimization
    void build_aabb_tree();
    AABB_Tree get_aabb_tree();
    void update_aabb_tree();

    // facture
    bool fracture;
    float roughness;
    void system_fracture(std::tuple<int, int, double, Eigen::Vector3cd>& break_point);
    void split(int point, glm::vec3& n, std::set<int>& split_pathway);
    void add_pt(int point);


private:
    // geometry parameter
    unsigned int numPoint;
    unsigned int numFace;
    unsigned int numTetre;
    std::vector<glm::vec3> vertex;
    std::vector<glm::vec3> position;
    std::vector<unsigned int> index;
    std::vector<unsigned int> face;
    std::vector<unsigned int> tetrahedra;
    std::vector<glm::vec3> normal;
    std::vector<glm::vec3> surface_normal;

    // integration parameter
    float dens;
    float timestep;
    bool use_gravity;
    float air_resistence;
    glm::vec3 gravity;
    glm::vec3 location;
    std::vector<float> mass;
    float lambda;
    float lambda_damping;
    float miu;
    float miu_damping;
    std::vector<glm::vec3> velocities;
    std::vector<glm::vec3> accelerations;
    std::vector<glm::vec3> object_collision_force;
    std::vector<glm::mat3> RestFrame;
    std::vector<glm::mat3> preEps;

    // shader parameter
    unsigned int vao;
    unsigned int vbo;
    unsigned int ebo;
    Shader m_system_shader;
    glm::mat4 transform;

    // collision data structure
    AABB_Tree aabb_tree;
    std::vector<CPM_GLM_AABB_NS::AABB> aabb_list;

    //light
    glm::vec3 lightPos;
    glm::vec3 viewPos;
    std::vector<glm::vec3> display;

};


#endif //PHYSICSSIMULATION_PHYSYS_H
