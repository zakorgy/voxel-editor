#version 450

layout(location = 0) in vec3 inPos;

layout(location = 1) in vec3 inOffset;

layout(set = 0, binding = 0) uniform LightUBO {
    vec4 pos;
    vec4 color;
    mat4 projection;
} light;

void main() { gl_Position = light.projection * vec4(inPos + inOffset, 1.0); }
