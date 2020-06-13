#version 450

layout(location = 0) in vec4 a_Pos;

layout(set = 0, binding = 0) uniform Locals {
    mat4 u_Transform;
};

void main() {;
    gl_Position = u_Transform * a_Pos;
}
