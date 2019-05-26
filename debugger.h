//
// Created by Xinming Zhang on 5/14/19.
//

#ifndef PHYSICSSIMULATION_DEBUGGER_H
#define PHYSICSSIMULATION_DEBUGGER_H

#include <iostream>
#include <vector>

#include "shader.h"
#include "physys.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class debugger {
public:
    debugger(MassSpringSystem& system);
    ~debugger()= default;

    void set_transform(glm::mat4 transform);
    void render();
    void update(MassSpringSystem& system);

private:
    std::vector<glm::vec3> normal_data;
    int index_num;
    void init_shader();
    std::vector<glm::vec3> pos;
    std::vector<glm::vec3> acc;
    std::vector<unsigned int> index;
    unsigned int vao, vbo, ebo;
    glm::mat4 transform;
    Shader shader;
};

class polyhedron{
public:
    polyhedron(std::vector<glm::vec3> polyhedron);
    ~polyhedron() = default;

    void set_transform(glm::mat4 transform);
    void render();
    void update();
private:
    void init_shader();
    std::vector<glm::vec3> point;
    int size;

};
#endif //PHYSICSSIMULATION_DEBUGGER_H
