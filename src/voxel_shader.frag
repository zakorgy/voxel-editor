#version 450

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inFragPos;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 1) uniform Light {
  vec4 lightPos;
  vec4 lighColor;
};

void main() {
  vec3 norm = normalize(inNormal);
  vec3 ambient = vec3(0.5);
  vec3 color = ambient;
  // Diffuse
  vec3 lightDir = normalize(lightPos.xyz - inFragPos.xyz);
  float diffuse = max(dot(norm, lightDir), 0.0);
  color += diffuse * lighColor.xyz;

  outColor = vec4(color, 1.0) * inColor;
}
