//
// Created by Xinming Zhang on 5/14/19.
//

#include "debugger.h"

#define VERTEX_SHADER ("shader/debug.vs")
#define FRAGMENT_SHADER ("shader/debug.fs")
#define GEOMETRY_SHADER ("shader/debug.gs")

debugger::debugger(MassSpringSystem& system) {
    this->index_num = int(system.getIndex().size());
    pos = system.getVertex();
    acc = system.getAcceleration();
    index = system.getIndex();
    init_shader();
}

void debugger::init_shader() {
    int size = pos.size();
    for(int i = 0; i < size; ++i){
        normal_data.push_back(pos[i]);
        normal_data.push_back(acc[i]);
    }

    shader = Shader(VERTEX_SHADER, FRAGMENT_SHADER, GEOMETRY_SHADER);
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size * 2 * sizeof(glm::vec3), &normal_data[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_num*sizeof(unsigned int), &index[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), 0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)sizeof(glm::vec3));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void debugger::set_transform(glm::mat4 transform) {
    this->transform = transform;
}

void debugger::render() {
    shader.use();
    shader.setMat4("transform", transform);
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, index_num, GL_UNSIGNED_INT, 0);
    shader.unuse();
}

void debugger::update(MassSpringSystem &system) {
    int size = pos.size();
    pos = system.getVertex();
    acc = system.getAcceleration();
    for(int i = 0; i < size; ++i){
        normal_data[2*i]   = pos[i];
        normal_data[2*i+1] = acc[i];
    }
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size * 2 * sizeof(glm::vec3), &normal_data[0], GL_STATIC_DRAW);
}

polyhedron::polyhedron(std::vector<glm::vec3> polyhedron) {

}