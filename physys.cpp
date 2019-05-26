//
// Created by Xinming Zhang on 5/7/19.
//
/**
 *  Integration module for simulation
 * */
#include "physys.h"


#define VERTEX_SHADER ("shader/particle.vs")
#define FRAGMENT_SHADER ("shader/particle.fs")

namespace P{
    float Lamba(float E, float v){
        return (E*v)/((1+v)*(1-2*v));
    }

    float Miu(float E, float v){
        return float(0.5) * E / (1+v);
    }
    // volume of tetrahedron
    float volume(glm::vec3& p0, glm::vec3& p1, glm::vec3& p2, glm::vec3& p3){
        glm::vec3 e1 = p2 - p3;
        glm::vec3 e2 = p1 - p3;
        glm::vec3 e3 = p0 - p3;

        float volume = glm::dot(glm::cross(e1, e2), e3) / 6.0;
        return volume;
    }
}

void MassSpringSystem::ComputeAccelerations(std::vector<glm::vec3> &acc) {
    if(!numPoint)
        return;
    acc.resize(numPoint);

    // gravity
    if(this->use_gravity)
        for(int i = 0; i < numPoint; ++i)
            acc[i] = gravity;

    // air resistence
    for (int j = 0; j < numPoint; ++j) {
        acc[j] += -air_resistence * velocities[j] / mass[j];
        acc[j] += accelerations[j] / mass[j];
        acc[j] += object_collision_force[j] / mass[j];
    }

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
        glm::mat3 d_eps = (eps - this->preEps[i]) * (1.0f/timestep);
        this->preEps[i] = eps;

        glm::mat3 elasticity = 2*miu*eps + lambda* (eps[0][0] + eps[1][1] + eps[2][2]) * glm::mat3(1.0);
        glm::mat3 damping = 2*miu_damping*d_eps + lambda_damping * (d_eps[0][0] + d_eps[1][1] + d_eps[2][2]) * glm::mat3(1.0);
        glm::mat3 Sigma = elasticity;

        glm::vec3 f1 = -F * Sigma * normal[4*i + 0] * 0.5f;
        glm::vec3 f3 = -F * Sigma * normal[4*i + 1] * 0.5f;
        glm::vec3 f2 = -F * Sigma * normal[4*i + 2] * 0.5f;
        glm::vec3 f4 = -F * Sigma * normal[4*i + 3] * 0.5f;

        //update acc
        acc[tetrahedra[4*i+0]] += f1 / mass[tetrahedra[4*i+0]];
        acc[tetrahedra[4*i+1]] += f2 / mass[tetrahedra[4*i+1]];
        acc[tetrahedra[4*i+2]] += f3 / mass[tetrahedra[4*i+2]];
        acc[tetrahedra[4*i+3]] += f4 / mass[tetrahedra[4*i+3]];
    }

    accelerations.assign(acc.begin(), acc.end());

}


unsigned int MassSpringSystem::GetNumPoint() {
    return numPoint;
}

unsigned int MassSpringSystem::GetNumTetra() {
    return numTetre;
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

void MassSpringSystem::SetAccelerations(const std::vector<glm::vec3> &acc) {
    accelerations.assign(acc.begin(), acc.end());
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
                                   float E,
                                   float v_damping,
                                   float E_damping,
                                   float timestep,
                                   bool use_gravity) {
    // parameter

    this->dens = densities;
    this->location = location;
    this->lambda = P::Lamba(E, v);
    this->miu = P::Miu(E, v);
    this->lambda_damping = P::Lamba(E_damping, v_damping);
    this->miu_damping = P::Miu(E_damping, v_damping);
    this->timestep = timestep;
    this->use_gravity = use_gravity;
    this->gravity = glm::vec3(0, -9.8, 0);
    this->air_resistence = 0.01;

    // mesh load
    setMesh(vertexfile, edgefile, tetrahedrafile);
    parameterInitialization();


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

void MassSpringSystem::normalCalculation() {
    for (int i = 0; i < numTetre; ++i) {
        // n1
        glm::vec3 n11(position[tetrahedra[4*i+1]] - position[tetrahedra[4*i+2]]);
        glm::vec3 n12(position[tetrahedra[4*i+3]] - position[tetrahedra[4*i+2]]);
        normal[4*i + 0] = glm::cross(n11, n12);
        // n3
        glm::vec3 n31(position[tetrahedra[4*i+0]] - position[tetrahedra[4*i+1]]);
        glm::vec3 n32(position[tetrahedra[4*i+3]] - position[tetrahedra[4*i+1]]);
        normal[4*i + 1] = glm::cross(n31, n32);

        // n2
        glm::vec3 n21(position[tetrahedra[4*i+2]] - position[tetrahedra[4*i+0]]);
        glm::vec3 n22(position[tetrahedra[4*i+3]] - position[tetrahedra[4*i+0]]);
        normal[4*i + 2] = glm::cross(n21, n22);
        // n4
        glm::vec3 n41(position[tetrahedra[4*i+1]] - position[tetrahedra[4*i+0]]);
        glm::vec3 n42(position[tetrahedra[4*i+2]] - position[tetrahedra[4*i+0]]);
        normal[4*i + 3] = glm::cross(n41, n42);
    }
}

void MassSpringSystem::parameterInitialization() {
    size_t t = vertex.size();
    preEps.resize(numTetre);
    for(int i = 0; i < numTetre; ++i){
        preEps[i] = glm::mat3(0.0);
    }
    // initialize mass
    mass.resize(numPoint);
    for(int i = 0; i < numPoint; ++i)
        mass[i] = 0;

    // intialize normal
    normal.resize(4*numTetre);
    for (int j = 0; j < numTetre; ++j) {
        normal[4*j+0] = glm::vec3(0);
    }

    massRCalculation();

    // initialize velocity
    velocities.resize(numPoint);
    for(int i = 0; i < numPoint; ++i){
        velocities[i] = glm::vec3(0);
    }
    velocities[0] = glm::vec3(15.0, 0, 0);
    position.assign(vertex.begin(), vertex.end());

    normalCalculation();


    // initialize acceleration
    this->accelerations.resize(numPoint);
    for(int i = 0; i < numPoint; ++i)
        accelerations[i] = glm::vec3(0);

    // initialize collision force
    this->object_collision_force.resize(numPoint);
    for(int i = 0; i < numPoint; ++i)
        object_collision_force[i] = glm::vec3(0);

    // building aabb tree structure
    aabb_tree = AABB_Tree(2*numTetre);
    build_aabb_tree();
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
            addVertex(atof(l2.c_str())+location.x, atof(l3.c_str())+location.y, atof(l4.c_str())+location.z);
        }

        vertex.erase(vertex.begin());
        numPoint = (int)vertex.size();
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
            addFace((unsigned int)atoi(l2.c_str()), (unsigned int)atoi(l3.c_str()), (unsigned int)atoi(l4.c_str()));
        }

        facestream.close();
        face.erase(face.begin(), face.begin()+3);

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "face file fail read" << std::endl;
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
            addTetra((unsigned int)atoi(l2.c_str()), (unsigned int)atoi(l3.c_str()), (unsigned int)atoi(l4.c_str()),(unsigned int)atoi(l5.c_str()));
        }
        tetrahedrastream.close();
        tetrahedra.erase(tetrahedra.begin(), tetrahedra.begin()+4);
        numTetre = int(tetrahedra.size() / 4);

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "tetra file fail read" << std::endl;
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

std::vector<unsigned int> MassSpringSystem::getTetra() {
    return tetrahedra;
}

void MassSpringSystem::get_tetra(unsigned int tetra_id, std::vector<glm::vec3>& tetra) {
    tetra.push_back(vertex[tetrahedra[4 * tetra_id + 0]]);
    tetra.push_back(vertex[tetrahedra[4 * tetra_id + 1]]);
    tetra.push_back(vertex[tetrahedra[4 * tetra_id + 2]]);
    tetra.push_back(vertex[tetrahedra[4 * tetra_id + 3]]);
}

std::vector<unsigned int> MassSpringSystem::getIndex() {
    return index;
}
std::vector<glm::vec3> MassSpringSystem::getVelocity() {
    return velocities;
}
std::vector<glm::vec3> MassSpringSystem::getAcceleration() {
    return accelerations;
}
std::vector<float> MassSpringSystem::getMass() {
    return mass;
}

// initialize shader program
void MassSpringSystem::initShader() {
    m_system_shader = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, GetNumPoint()*sizeof(glm::vec3), &getVertex()[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, int(getIndex().size())*sizeof(float), &getIndex()[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
//    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)(mesh.GetNumPoint()*sizeof(glm::vec3)));
    glEnableVertexAttribArray(0);
//    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void MassSpringSystem::set_transformation(glm::mat4 transform) {
    this->transform = transform;
}

void MassSpringSystem::render_system() {
    m_system_shader.use();
    m_system_shader.setMat4("transform", transform);
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, (int)getIndex().size(), GL_UNSIGNED_INT, 0);
    m_system_shader.unuse();
}
void MassSpringSystem::delete_shader() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, & vbo);
    glDeleteBuffers(1, &ebo);
}

void MassSpringSystem::update() {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, GetNumPoint()*sizeof(glm::vec3), &getVertex()[0], GL_STATIC_DRAW);
}


// build aabb tree from aabb list
void MassSpringSystem::build_aabb_tree() {
    for (int i = 0; i < numTetre; ++i) {
        glm::vec3 p1 = position[tetrahedra[4*i + 0]];
        glm::vec3 p2 = position[tetrahedra[4*i + 1]];
        glm::vec3 p3 = position[tetrahedra[4*i + 2]];
        glm::vec3 p4 = position[tetrahedra[4*i + 3]];

        Tetrahedra tet(p1, p2, p3, p4);
        tet.ID = i;
        aabb_tree.insertObject(tet);
    }
}

AABB_Tree MassSpringSystem::get_aabb_tree() {
    return aabb_tree;
}
// update aabb tree
void MassSpringSystem::update_aabb_tree() {
    for(int i = 0; i < numTetre; ++i){
        glm::vec3 p1 = vertex[tetrahedra[4*i + 0]];
        glm::vec3 p2 = vertex[tetrahedra[4*i + 1]];
        glm::vec3 p3 = vertex[tetrahedra[4*i + 2]];
        glm::vec3 p4 = vertex[tetrahedra[4*i + 3]];

        Tetrahedra new_tet(p1, p2, p3, p4);
        new_tet.ID = i;
        aabb_tree.updateObject(new_tet);
    }
}

// process object collision force on each point of corresponding tetrahedron
void MassSpringSystem::set_collision_force(unsigned int tetra_id, glm::vec3& force) {



    object_collision_force[tetrahedra[4*tetra_id + 0]] += force;
    object_collision_force[tetrahedra[4*tetra_id + 1]] += force;
    object_collision_force[tetrahedra[4*tetra_id + 2]] += force;
    object_collision_force[tetrahedra[4*tetra_id + 3]] += force;




//    velocities[tetrahedra[4*tetra_id + argMax]] = glm::vec3(0);



}

// before each iteration, clear collision force for each object
void MassSpringSystem::collision_force_clear() {
    for (int i = 0; i < numPoint; ++i) {
        object_collision_force[i] = glm::vec3(0);
    }
}