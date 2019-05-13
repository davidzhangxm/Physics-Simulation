#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aAcc;

uniform mat4 transform;
uniform mat4 Rotation;

out vec4 acc;

void main(){
    gl_Position = transform * vec4(aPos.x, aPos.y, aPos.z, 1.0);
    acc = transform * vec4(aAcc, 0.0);
}
