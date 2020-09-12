#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec3 inNormal;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec3 fragViewVec;
layout(location = 3) out vec3 fragLightVec;

layout(set = 0, binding = 0) uniform UBO {
    mat4 model;
    mat4 view;
    mat4 projection;
} ubo;

layout(set = 0, binding = 1) uniform LightUBO {
    vec3 lightPos;
    vec4 lighColor;
} light;

void main() {
    gl_Position = ubo.projection * ubo.view * ubo.model * vec4(inPos, 1.0);
    vec4 worldPos = ubo.model * vec4(inPos, 1.0);

    fragColor = inColor;
    fragNormal = mat3(ubo.model) * inNormal;
    fragViewVec = (ubo.view * worldPos).xyz;
    fragLightVec = light.lightPos - vec3(worldPos);
}
