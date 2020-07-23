#version 450

layout(location = 0) in vec4 fragColor;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec3 fragViewVec;
layout(location = 3) in vec3 fragLightVec;
layout(location = 4) in vec4 vertPos;
layout(location = 5) in mat4 fragLightProj;


layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 2) uniform texture2D t_Shadow;
layout(set = 0, binding = 3) uniform samplerShadow s_Shadow;

float fetch_shadow(vec4 homogeneous_coords) {
  if (homogeneous_coords.w <= 0.0) {
    return 1.0;
  }
  // compensate for the Y-flip difference between the NDC and texture
  // coordinates
  const vec2 flip_correction = vec2(0.5, -0.5);
  // compute texture coordinates for shadow lookup
  vec3 light_local =
      vec3(homogeneous_coords.xy * flip_correction / homogeneous_coords.w + 0.5,
           homogeneous_coords.z / homogeneous_coords.w);

  float bias = 0.7;
  vec3 ShadowCoord = vec3(light_local.xy, light_local.z);
  float shadow = texture( sampler2DShadow(t_Shadow, s_Shadow), ShadowCoord);
  if ( shadow  <  (ShadowCoord.z - bias)){
    return shadow;
  }
  return 1.0;
}

void main() {
  vec3 N = normalize(fragNormal);
  vec3 L = normalize(fragLightVec);
  vec3 V = normalize(fragViewVec.xyz);
  vec3 R = reflect(L, N);

  vec3 ambient = fragColor.xyz * 0.2;
  vec3 diffuse = fragColor.xyz * max(dot(N, L), 0.0);
  //vec3 specular = pow(max(dot(R, V), 0.0), 16.0) * vec3(1.35);
  float shadow = fetch_shadow(fragLightProj * vertPos);

  outColor = vec4(ambient + diffuse * shadow /*+ specular*/, fragColor.z);
}
