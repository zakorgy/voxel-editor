#version 450

layout(location = 0) in vec4 inPos;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform Locals {
    mat4 u_Transform;
};

void main() {;
    gl_Position = u_Transform * inPos;
    outColor = inColor;
}
