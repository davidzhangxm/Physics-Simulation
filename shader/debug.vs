# version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out VS_OUT {
    vec3 normal;
} vs_out;

uniform mat4 transform;

void main() {
    gl_Position = transform * vec4(aPos, 1.0);
    vs_out.normal = vec3(transform * vec4(aNormal, 0.0));
}
