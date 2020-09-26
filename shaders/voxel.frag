#version 450

layout(location = 0) in vec4 fragColor;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec3 fragViewVec;
layout(location = 3) in vec3 fragLightVec;
layout(location = 4) in vec4 vertPos;
layout(location = 5) in mat4 fragLightProj;


layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 2) uniform texture2D t_Shadow;
layout(set = 0, binding = 3) uniform sampler s_Shadow;

float ShadowCalculation(vec4 fragPosLightSpace)
{
    const vec2 flip_correction = vec2(0.5, -0.5);
    // compute texture coordinates for shadow lookup
    vec3 projCoords =
      vec3(fragPosLightSpace.xy * flip_correction / fragPosLightSpace.w + 0.5,
           fragPosLightSpace.z / fragPosLightSpace.w);

     if (projCoords.z > 1.0) {
        return 0.0;
     }
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(sampler2D(t_Shadow, s_Shadow), projCoords.xy).r;
    // get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;
    // check whether current frag pos is in shadow
    float shadow = currentDepth > closestDepth  ? 0.5 : 0.0;

    return shadow;
}

void main() {
  vec3 N = normalize(fragNormal);
  vec3 L = normalize(fragLightVec);
  vec3 V = normalize(fragViewVec.xyz);
  vec3 R = reflect(L, N);

  vec3 ambient = fragColor.xyz * 0.2;
  vec3 diffuse = fragColor.xyz * max(dot(N, L), 0.0);
  //vec3 specular = pow(max(dot(R, V), 0.0), 16.0) * vec3(1.35);
  vec4 FragPosLightSpace = fragLightProj * vertPos;
  float shadow = ShadowCalculation(FragPosLightSpace);
  vec3 lighting = (ambient + (1.0 - shadow) * (diffuse /*+ specular*/));

  outColor = vec4(lighting, fragColor.z);
}
