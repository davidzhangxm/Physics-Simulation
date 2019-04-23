//
// Created by Xinming Zhang on 4/18/19.
//

#ifndef PHYSICSSIMULATION_MESH_H
#define PHYSICSSIMULATION_MESH_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


class Mesh{
public:
    Mesh(const char* vertexfile, const char* edgefile, const char* tetrahedrafile);
    ~Mesh(){}

    void setMesh(const char* vertexfile, const char* edgefile, const char* tetrahedrafile);
    void readVertex(const char* vertexfile);
    void readEdgefile(const char* edgefile);
    void readTetrafile(const char* tetrahedrafile);
    void addVertex(float x, float y, float z);
    void addIndex(unsigned int a, unsigned int b, unsigned int c);
    void addTetra(unsigned int a, unsigned int b, unsigned int c, unsigned int d);

    std::vector<float> getVertex();
    std::vector<unsigned int> getIndex();

private:
    std::vector<float> vertex;
    std::vector<unsigned int> index;
    std::vector<unsigned int> tetrahedra;
};

Mesh::Mesh(const char *vertexfile, const char *edgefile, const char *tetrahedrafile) {
    setMesh(vertexfile, edgefile, tetrahedrafile);
}

void Mesh::setMesh(const char *vertexfile, const char *edgefile, const char *tetrahedrafile) {
    readVertex(vertexfile);
    readEdgefile(edgefile);
    readTetrafile(tetrahedrafile);
}

void Mesh::readVertex(const char *vertexfile) {
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
            addVertex(atof(l2.c_str()), atof(l3.c_str()), atof(l4.c_str()));
        }

//        vertex.erase(vertex.begin(), vertex.begin()+3);
        vertexstream.close();

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "vertex file fail read" << std::endl;
    }
}

void Mesh::readEdgefile(const char *edgefile) {
    std::ifstream edgestream;
    char edgeline[100] = {0};
    edgestream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    const char* edgepath = edgefile;

    std::string l1, l2, l3, l4;
    try
    {
        edgestream.open(edgepath);
        while(edgestream.getline(edgeline, sizeof(edgeline))){
            std::stringstream linestream(edgeline);
            linestream >> l1;
            if(l1=="#")
                break;
            linestream >> l2;
            linestream >> l3;
            linestream >> l4;
            addIndex((unsigned int)atoi(l2.c_str()), (unsigned int)atoi(l3.c_str()), (unsigned int)atoi(l4.c_str()));
        }

        edgestream.close();
        index.erase(index.begin(), index.begin()+3);

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "vertex file fail read" << std::endl;
    }

}

void Mesh::readTetrafile(const char *tetrahedrafile) {
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

    }
    catch(std::ifstream::failure e)
    {
        std::cout << "vertex file fail read" << std::endl;
    }
}

void Mesh::addVertex(float x, float y, float z) {
    vertex.push_back(x);
    vertex.push_back(y);
    vertex.push_back(z);
}

void Mesh::addIndex(unsigned int a, unsigned int b, unsigned int c) {
    index.push_back(a);
    index.push_back(b);
    index.push_back(c);
}

void Mesh::addTetra(unsigned int a, unsigned int b, unsigned int c, unsigned int d) {
    tetrahedra.push_back(a);
    tetrahedra.push_back(b);
    tetrahedra.push_back(c);
    tetrahedra.push_back(d);

}

std::vector<float> Mesh::getVertex() {
    return vertex;
}

std::vector<unsigned int> Mesh::getIndex() {
    return index;
}

#endif //PHYSICSSIMULATION_MESH_H
