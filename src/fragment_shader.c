varying vec3 P;
varying vec3 N;
varying vec3 I;

void main()
{

  vec3 L = normalize(gl_LightSource[0].position.xyz - P);
  vec3 E = normalize(P); // we are in Eye Coordinates, so EyePos is (0,0,0)
  vec3 R = normalize(-reflect(P,N));

  //calculate Ambient Term:
  vec4 Iamb = gl_FrontLightProduct[0].ambient;

  //calculate Diffuse Term:
  vec4 Idiff = gl_FrontLightProduct[0].diffuse * max(dot(N,L), 0.0);
  Idiff = clamp(Idiff, 0.0, 1.0);

  // calculate Specular Term:
  vec4 Ispec = gl_FrontLightProduct[0].specular
    * pow(max(dot(R,E),0.0),1);
  Ispec = clamp(Ispec, 0.0, 1.0);

  // write Total Color:
  gl_FragColor = gl_FrontLightModelProduct.sceneColor + Iamb + Idiff + Ispec;

}
