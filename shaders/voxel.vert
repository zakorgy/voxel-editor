#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;

layout(location = 2) in vec3 inOffset;
layout(location = 3) in vec4 inColor;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec3 fragViewVec;
layout(location = 3) out vec3 fragLightVec;
layout(location = 4) out vec4 vertPos;
layout(location = 5) out mat4 fragLightProj;

layout(set = 0, binding = 0) uniform UBO {
    mat4 model;
    mat4 view;
    mat4 projection;
} ubo;

layout(set = 0, binding = 1) uniform LightUBO {
    vec4 direction;
    vec4 color;
    mat4 projection;
} light;

void main() {
    vec4 pos = vec4(inPos + inOffset, 1.0);
    gl_Position = ubo.projection * ubo.view * ubo.model * pos;
    vec4 worldPos = ubo.model * pos;

    fragColor = inColor;
    fragNormal = mat3(ubo.model) * inNormal;
    fragViewVec = (ubo.view * worldPos).xyz;
    fragLightVec = light.direction.xyz;
    vertPos = pos;
    fragLightProj = light.projection;
}
