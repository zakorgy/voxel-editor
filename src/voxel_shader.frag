#version 450

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inFragPos;

layout(location = 0) out vec4 outColor;

const vec3 LIGHT_COLOR = vec3(1.0, 1.0, 1.0);
const float AMBIENT_STRENGTH = 0.2;
const float SPECULAR_STRENGTH = 0.8;

void main() {
  vec3 lightPos = vec3(16.0, 24.0, 8.0); // camerPos - vec3(4.0);
  // Ambient
  vec3 ambient = AMBIENT_STRENGTH * LIGHT_COLOR;
  // Diffuse
  vec3 norm = normalize(inNormal);
  vec3 lightDir = normalize(lightPos - inFragPos);
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diff * LIGHT_COLOR;
  // Specular
  vec3 viewDir = normalize(lightPos - inFragPos);
  vec3 reflectDir = reflect(-lightDir, norm);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
  vec3 specular = SPECULAR_STRENGTH * spec * LIGHT_COLOR;

  outColor = vec4((ambient + diffuse + specular) * inColor.xyz, inColor.w);
}
