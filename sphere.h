//
// Created by Xinming Zhang on 4/17/19.
//

#ifndef PHYSICSSIMULATION_SPHERE_H
#define PHYSICSSIMULATION_SPHERE_H

#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


class Sphere{
public:
    Sphere(float radius, int sectors, int stacks, glm::vec3 center);
    ~Sphere(){}

    void set(float radius, int sectors, int stacks, glm::vec3 center);
    void genVertices();
    void clearArrays();
    void addVertex(float x, float y, float z);
    void addIndex(unsigned int a, unsigned int b, unsigned int c);

    std::vector<float> getVertex();
    std::vector<unsigned int> getIndex();
private:
    float radius;
    int sectorCount;
    int stackCount;
    glm::vec3 center;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
};

Sphere::Sphere(float radius, int sectors, int stacks, glm::vec3 center) {
    set(radius,sectors, stacks, center);
    genVertices();
}

void Sphere::set(float radius, int sectors, int stacks, glm::vec3 center) {
    this->radius = radius;
    this->sectorCount = sectors;
    this->stackCount = stacks;
    this->center = center;
}

void Sphere::clearArrays() {
    vertices.clear();
    indices.clear();
}

void Sphere::addVertex(float x, float y, float z) {
    vertices.push_back(x);
    vertices.push_back(y);
    vertices.push_back(z);
}
void Sphere::addIndex(unsigned int a, unsigned int b, unsigned int c) {
    indices.push_back(a);
    indices.push_back(b);
    indices.push_back(c);
}

void Sphere::genVertices() {
    const float PI = 3.1415926;
    clearArrays();
    float x, y, z, xy;                              // vertex position

    float sectorStep = 2 * PI / sectorCount;
    float stackStep = PI / stackCount;
    float sectorAngle, stackAngle;

    for(int i = 0; i <= stackCount; ++i)
    {
        stackAngle = PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
        xy = radius * cosf(stackAngle);             // r * cos(u)
        z = radius * sinf(stackAngle);              // r * sin(u)

        // add (sectorCount+1) vertices per stack
        // the first and last vertices have same position and normal, but different tex coords
        for(int j = 0; j <= sectorCount; ++j)
        {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

            // vertex position
            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)
            addVertex(x+center.x, y+center.y, z+center.z);

            // normalized vertex normal
//            nx = x * lengthInv;
//            ny = y * lengthInv;
//            nz = z * lengthInv;
//            addNormal(nx, ny, nz);

//            // vertex tex coord between [0, 1]
//            s = (float)j / sectorCount;
//            t = (float)i / stackCount;
//            addTexCoord(s, t);
        }
    }

    // indices
    //  k1--k1+1
    //  |  / |
    //  | /  |
    //  k2--k2+1
    unsigned int k1, k2;
    for(int i = 0; i < stackCount; ++i)
    {
        k1 = i * (sectorCount + 1);     // beginning of current stack
        k2 = k1 + sectorCount + 1;      // beginning of next stack

        for(int j = 0; j < sectorCount; ++j, ++k1, ++k2)
        {
            // 2 triangles per sector excluding 1st and last stacks
            if(i != 0)
            {
                addIndex(k1, k2, k1+1);   // k1---k2---k1+1
            }

            if(i != (stackCount-1))
            {
                addIndex(k1+1, k2, k2+1); // k1+1---k2---k2+1
            }

            // vertical lines for all stacks
//            lineIndices.push_back(k1);
//            lineIndices.push_back(k2);
//            if(i != 0)  // horizontal lines except 1st stack
//            {
//                lineIndices.push_back(k1);
//                lineIndices.push_back(k1 + 1);
//            }
        }
    }
}

std::vector<float> Sphere::getVertex() {
    return vertices;
}

std::vector<unsigned int> Sphere::getIndex() {
    return indices;
}

#endif //PHYSICSSIMULATION_SPHERE_H
