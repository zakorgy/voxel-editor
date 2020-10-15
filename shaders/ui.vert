#version 450

const vec2 pos_transform[3] = vec2[](vec2(0.0, -0.5), vec2(0.5), vec2(-0.5, 0.5));

void main()
{
  gl_Position = vec4(pos_transform[gl_VertexIndex], 0.0, 1.0);
}
