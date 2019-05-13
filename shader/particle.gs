#version 330 core

layout (points) in;
layout (lines, max_vertices = 2) out;

in vec4 acc;

void main() {
    gl_Position = gl_Position;
    EmitVertex();

    gl_Position = gl_Position + acc * 10;
    EmitVertex();
    EndPrimitive();
}