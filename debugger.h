//
// Created by Xinming Zhang on 5/14/19.
//

#ifndef PHYSICSSIMULATION_DEBUGGER_H
#define PHYSICSSIMULATION_DEBUGGER_H

#include <iostream>
#include <vector>

#include "shader.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class debugger {
public:
    debugger(std::vector<glm::vec3> &position, std::vector<glm::vec3> &acceleration);
    ~debugger()= default;

private:
    void init_shader();
    std::vector<glm::vec3> pos;
    std::vector<glm::vec3> acc;
    unsigned int vao, vbo, ebo;
    Shader shader;
};


#endif //PHYSICSSIMULATION_DEBUGGER_H
