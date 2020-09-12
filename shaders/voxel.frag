#version 450

layout(location = 0) in vec4 fragColor;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec3 fragViewVec;
layout(location = 3) in vec3 fragLightVec;

layout(location = 0) out vec4 outColor;

void main() {
  vec3 N = normalize(fragNormal);
  vec3 L = normalize(fragLightVec);
  vec3 V = normalize(fragViewVec);
  vec3 R = reflect(L, N);

  vec3 ambient = fragColor.xyz * 0.2;
  vec3 diffuse = fragColor.xyz * max(dot(N, L), 0.0);
  //vec3 specular = pow(max(dot(R, V), 0.0), 16.0) * vec3(1.35);

  outColor = vec4(ambient + diffuse /*+ specular*/, fragColor.z);
}
