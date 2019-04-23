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

glm::vec3 gravity = glm::vec3(0, -9.8, 0);

float Lamba(float E, float v){
    return (E*v)/((1+v)*(1-2*v));
}

float Miu(float E, float v){
    return float(0.5) * E / (1+v);
}


class PhysicsSystem{
public:
//    virtual int GetNumDofs();
    virtual void GetPositions(std::vector<glm::vec3> &pos){}
    virtual void GetVelocities(std::vector<glm::vec3> &vel){}
    virtual int GetNumPoint(){return 0;}

    virtual void SetPositions(const std::vector<glm::vec3> &pos){}
    virtual void SetVelocities(const std::vector<glm::vec3> &vel){}
    virtual void ComputeAccelerations(std::vector<glm::vec3> &acc){}
};

class MassSpringSystem: public PhysicsSystem{
public:
    MassSpringSystem(const char* vertexfile,
                     const char* edgefile,
                     const char* tetrahedrafile,
                     float densities,
                     glm::vec3 location,
                     float v,
                     float E);
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
    int  GetNumPoint();

    void SetPositions(const std::vector<glm::vec3> &pos);
    void SetVelocities(const std::vector<glm::vec3> &vel);
    void ComputeAccelerations(std::vector<glm::vec3> &acc);


    std::vector<glm::vec3> getVertex();
    std::vector<unsigned int> getIndex();
    std::vector<glm::vec3> getVelocity();

private:
    int numPoint;
    int numTetre;
    std::vector<glm::vec3> vertex;
    std::vector<unsigned int> index;
    std::vector<unsigned int> face;
    std::vector<unsigned int> tetrahedra;

    float dens;
    glm::vec3 location;
    std::vector<float> mass;
    float v;
    float E;
    std::vector<glm::vec3> velocities;
    std::vector<glm::mat3> RestFrame;
};

/**
 *  Integration module for simulation
 * */



void MassSpringSystem::ComputeAccelerations(std::vector<glm::vec3> &acc) {
    std::vector<glm::vec3> atmp(numPoint, glm::vec3(0.0));
    acc.assign(atmp.begin(), atmp.end());

    float lamda = Lamba(E, v);
    float miu = Miu(E, v);

    float K[6][6] = {
            2*miu+lamda,  lamda,       lamda,       0,  0, 0,
            lamda,        2*miu+lamda, lamda,       0,  0, 0,
            lamda,        lamda,       2*miu+lamda, 0,  0, 0,
            0,            0,           0,           miu,0, 0,
            0,            0,           0,           0,  miu,0,
            0,            0,           0,           0,  0, miu
    };

    for(int i = 0; i < numTetre; ++i){
        glm::vec3 ne1 = vertex[tetrahedra[4*i+2]] - vertex[tetrahedra[4*i+3]];
        glm::vec3 ne2 = vertex[tetrahedra[4*i+1]] - vertex[tetrahedra[4*i+3]];
        glm::vec3 ne3 = vertex[tetrahedra[4*i+0]] - vertex[tetrahedra[4*i+3]];

        float t[] = {
                ne1.x, ne1.y, ne1.z,
                ne2.x, ne2.y, ne2.z,
                ne3.x, ne3.y, ne3.z
        };
        glm::mat3 T = glm::make_mat3(t);
        glm::mat3 F = T * this->RestFrame[i];
        glm::mat3 eps = float(0.5) * (glm::transpose(F) * F - glm::mat3(1.0));
        float Veps[] = {
                eps[0][0], eps[1][1], eps[2][2], 2*eps[2][1], 2*eps[2][0], 2*eps[1][0]
        };

        float sigma[6] = {0 , 0, 0, 0, 0,0};
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                sigma[i] += K[i][j] * Veps[j];
            }
        }

        float sigma_matrix [] = {
                sigma[0], sigma[5], sigma[4],
                sigma[5], sigma[1], sigma[3],
                sigma[4], sigma[3], sigma[2]
        };
        glm::mat3 Sigma = glm::make_mat3(sigma_matrix);

        // n1
        glm::vec3 n11(vertex[tetrahedra[4*i+2]] - vertex[tetrahedra[4*i+3]]);
        glm::vec3 n12(vertex[tetrahedra[4*i+1]] - vertex[tetrahedra[4*i+3]]);
        glm::vec3 f1 = -float(0.5) * glm::cross(n11, n12) * Sigma;
        // n3
        glm::vec3 n31(vertex[tetrahedra[4*i+1]] - vertex[tetrahedra[4*i+3]]);
        glm::vec3 n32(vertex[tetrahedra[4*i+0]] - vertex[tetrahedra[4*i+3]]);
        glm::vec3 f3 = -float(0.5) * glm::cross(n31, n32) * Sigma;

        // n2
        glm::vec3 n21(vertex[tetrahedra[4*i+2]] - vertex[tetrahedra[4*i+0]]);
        glm::vec3 n22(vertex[tetrahedra[4*i+3]] - vertex[tetrahedra[4*i+0]]);
        glm::vec3 f2 = -float(0.5) * glm::cross(n21, n22) * Sigma;
        // n4
        glm::vec3 n41(vertex[tetrahedra[4*i+1]] - vertex[tetrahedra[4*i+0]]);
        glm::vec3 n42(vertex[tetrahedra[4*i+2]] - vertex[tetrahedra[4*i+0]]);
        glm::vec3 f4 = -float(0.5) * glm::cross(n41, n42) * Sigma;

        //update acc
        acc[tetrahedra[4*i+0]] += f1 / mass[tetrahedra[4*i+0]];
        acc[tetrahedra[4*i+1]] += f2 / mass[tetrahedra[4*i+1]];
        acc[tetrahedra[4*i+2]] += f3 / mass[tetrahedra[4*i+2]];
        acc[tetrahedra[4*i+3]] += f4 / mass[tetrahedra[4*i+3]];
    }

    // gravity
//    acc[0] = glm::vec3(0, 0, 0);
    for (int k = 0; k < numPoint; ++k) {
        acc[k] += gravity;
    }

    return;
}

int MassSpringSystem::GetNumPoint() {
    return numPoint;
}

void MassSpringSystem::GetPositions(std::vector<glm::vec3> &pos) {
    pos.assign(vertex.begin(), vertex.end());
}

void MassSpringSystem::GetVelocities(std::vector<glm::vec3> &vel) {
    vel.assign(velocities.begin(), velocities.end());
}

void MassSpringSystem::SetPositions(const std::vector<glm::vec3> &pos) {
    vertex.assign(pos.begin(), pos.end());
}

void MassSpringSystem::SetVelocities(const std::vector<glm::vec3> &vel) {
    velocities.assign(vel.begin(), vel.end());
}

/**
 *   Intiliazation module for the model
 *
 * */

MassSpringSystem::MassSpringSystem(const char *vertexfile,
                                   const char *edgefile,
                                   const char *tetrahedrafile,
                                   float densities,
                                   glm::vec3 location,
                                   float v,
                                   float E) {
    // parameter

    this->v = v;
    this->E = E;
    this->dens = densities;
    this->location = location;

    // mesh load
    setMesh(vertexfile, edgefile, tetrahedrafile);
    std::vector<float> tmp(numPoint,0);

    mass.clear();
    mass.assign(tmp.begin(), tmp.end());

    massRCalculation();
    velocityInitialization();


}
void MassSpringSystem::setMesh(const char *vertexfile, const char *facefile, const char *tetrahedrafile) {
    readVertex(vertexfile);
    readFacefile(facefile);
    readTetrafile(tetrahedrafile);
    readIndex(tetrahedra);
}

void MassSpringSystem::massRCalculation() {
    for(int i = 0; i<numTetre; ++i){
        glm::vec3 e1 = vertex[tetrahedra[4*i+2]] - vertex[tetrahedra[4*i+3]];
        glm::vec3 e2 = vertex[tetrahedra[4*i+1]] - vertex[tetrahedra[4*i+3]];
        glm::vec3 e3 = vertex[tetrahedra[4*i+0]] - vertex[tetrahedra[4*i+3]];

        float vi = glm::dot(glm::cross(e1, e2), e3) / 6.0;
        float mi = vi * dens;
        mass[tetrahedra[4*i+0]] += mi/4;
        mass[tetrahedra[4*i+1]] += mi/4;
        mass[tetrahedra[4*i+2]] += mi/4;
        mass[tetrahedra[4*i+3]] += mi/4;

        float r[] = {
                e1.x, e1.y, e1.z,
                e2.x, e2.y, e2.z,
                e3.x, e3.y, e3.z
        };
        glm::mat3 R = glm::make_mat3(r);
        RestFrame.push_back(glm::inverse(R));
    }
}

void MassSpringSystem::velocityInitialization() {
    std::vector<glm::vec3> vtmp(numPoint, glm::vec3(0, 0, 0));
    for (int i = 0; i < numPoint; ++i) {
        vtmp[i] = glm::vec3(-0.2,0,0);
    }
//    vtmp[0] = glm::vec3(-0.2, 0, 0);
    velocities.clear();
    velocities.assign(vtmp.begin(), vtmp.end());
}

void MassSpringSystem::readVertex(const char *vertexfile) {
    std::ifstream vertexstream;

    char vertexline[100] = {0};

    vertexstream.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    const char* vertexpath = vertexfile;

    std::string l1, l2, l3, l4;
    try
    {
        vertexstream.open(vertexpath);
        while(vertexstream.getline(vertexline, sizeof(vertexline))){
            std::stringstream linestream(vertexline);
            linestream >> l1;
            if(l1=="#")
                break;
            linestream >> l2;
            linestream >> l3;
            linestream >> l4;
            addVertex(atof(l2.c_str())/100.0+location.x, atof(l3.c_str())/100.0+location.y, atof(l4.c_str())/100.0+location.z);
        }

        vertex.erase(vertex.begin());
        numPoint = int(vertex.size());
        vertexstream.close();

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "vertex file fail read" << std::endl;
    }
}

void MassSpringSystem::readFacefile(const char *facefile) {
    std::ifstream facestream;
    char faceline[100] = {0};
    facestream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    const char* facepath = facefile;

    std::string l1, l2, l3, l4;
    try
    {
        facestream.open(facepath);
        while(facestream.getline(faceline, sizeof(faceline))){
            std::stringstream linestream(faceline);
            linestream >> l1;
            if(l1=="#")
                break;
            linestream >> l2;
            linestream >> l3;
            linestream >> l4;
            addFace((unsigned int)atoi(l2.c_str())-1, (unsigned int)atoi(l3.c_str())-1, (unsigned int)atoi(l4.c_str())-1);
        }

        facestream.close();
        face.erase(face.begin(), face.begin()+3);

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "vertex file fail read" << std::endl;
    }

}

void MassSpringSystem::readTetrafile(const char *tetrahedrafile) {
    std::ifstream tetrahedrastream;
    char tetraline[100] = {0};
    tetrahedrastream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    const char* tetrapath = tetrahedrafile;

    std::string l1, l2, l3, l4, l5;
    try
    {
        tetrahedrastream.open(tetrapath);
        while(tetrahedrastream.getline(tetraline, sizeof(tetraline))){
            std::stringstream linestream(tetraline);
            linestream >> l1;
            if(l1=="#")
                break;
            linestream >> l2;
            linestream >> l3;
            linestream >> l4;
            linestream >> l5;
            addTetra((unsigned int)atoi(l2.c_str())-1, (unsigned int)atoi(l3.c_str())-1, (unsigned int)atoi(l4.c_str())-1,(unsigned int)atoi(l5.c_str())-1);
        }
        tetrahedrastream.close();
        tetrahedra.erase(tetrahedra.begin(), tetrahedra.begin()+4);
        numTetre = int(tetrahedra.size())/4;

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "vertex file fail read" << std::endl;
    }
}

void MassSpringSystem::readIndex(std::vector<unsigned int> tetrahedra) {
    for (int i = 0; i < numTetre; ++i) {
        addIndex(tetrahedra[4*i+0],tetrahedra[4*i+1],tetrahedra[4*i+2]);
        addIndex(tetrahedra[4*i+0],tetrahedra[4*i+1],tetrahedra[4*i+3]);
        addIndex(tetrahedra[4*i+0],tetrahedra[4*i+2],tetrahedra[4*i+3]);
        addIndex(tetrahedra[4*i+1],tetrahedra[4*i+2],tetrahedra[4*i+3]);

    }
}

void MassSpringSystem::addVertex(float x, float y, float z) {
    vertex.push_back(glm::vec3(x, y, z));
}

void MassSpringSystem::addFace(unsigned int a, unsigned int b, unsigned int c) {
    face.push_back(a);
    face.push_back(b);
    face.push_back(c);
}

void MassSpringSystem::addTetra(unsigned int a, unsigned int b, unsigned int c, unsigned int d) {
    tetrahedra.push_back(a);
    tetrahedra.push_back(b);
    tetrahedra.push_back(c);
    tetrahedra.push_back(d);
}
void MassSpringSystem::addIndex(unsigned int a, unsigned int b, unsigned int c) {
    index.push_back(a);
    index.push_back(b);
    index.push_back(c);
}

std::vector<glm::vec3> MassSpringSystem::getVertex() {
    return vertex;
}

std::vector<unsigned int> MassSpringSystem::getIndex() {
    return index;
}
std::vector<glm::vec3> MassSpringSystem::getVelocity() {
    return velocities;
}
#endif //PHYSICSSIMULATION_PHYSYS_H
