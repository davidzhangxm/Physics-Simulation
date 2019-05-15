//
// Created by Xinming Zhang on 5/14/19.
//

#include "debugger.h"

#define VERTEX_SHADER ("shader/")
#define FRAGMENT_SHADER ("shader/")

debugger::debugger(std::vector<glm::vec3> &position, std::vector<glm::vec3> &acceleration) {
    pos.assign(position.begin(), position.end());
    acc.assign(acceleration.begin(), acceleration.end());
    init_shader();
}

//void debugger::init_shader() {
//    shader = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
//    glGenVertexArrays(1, &vao);
//    glGenBuffers(1, &vbo);
//    glGenBuffers(1, &ebo);
//
//    glBindVertexArray(vao);
//    glBindBuffer(GL_ARRAY_BUFFER, vbo);
//    glBufferData(GL_ARRAY_BUFFER, )
//}