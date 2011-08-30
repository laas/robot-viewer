attribute vec2 aTextureCoord;

varying vec3 vTransformedNormal;
varying vec4 vPosition;
varying vec2 vTextureCoord;

void main()
{
  //Transform vertex by modelview and projection matrices
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  // Position in clip space
  vPosition = gl_ModelViewMatrix * gl_Vertex;

  // Normal transform (transposed model-view inverse)
  vTransformedNormal = gl_NormalMatrix * gl_Normal;

  // Forward current color and texture coordinates after applying texture matrix
  gl_FrontColor = gl_Color;
  vTextureCoord = aTextureCoord;

}
