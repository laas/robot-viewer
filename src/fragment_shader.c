#ifdef GL_ES
precision highp float;
#endif

varying vec2 vTextureCoord;
varying vec3 vTransformedNormal;
varying vec4 vPosition;

uniform vec3 uMaterialAmbientColor;
uniform vec3 uMaterialDiffuseColor;
uniform vec3 uMaterialSpecularColor;
uniform float uMaterialShininess;
uniform vec3 uMaterialEmissiveColor;

uniform bool uShowSpecularHighlights;
uniform bool uUseTextures;

uniform vec3 uAmbientLightingColor;

uniform vec3 uPointLightingLocation;
uniform vec3 uPointLightingDiffuseColor;
uniform vec3 uPointLightingSpecularColor;

uniform sampler2D uSampler;

uniform bool uModernShader;

void legacy_main(void) {
  // Phong shader http://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/lighting.php
  vec3 v = vec3(vPosition);
  vec3 N = vTransformedNormal;

  vec3 L = normalize(gl_LightSource[0].position.xyz - v);
  vec3 E = normalize(-v); // we are in Eye Coordinates, so EyePos is (0,0,0)
  vec3 R = normalize(-reflect(L,N));

  //calculate Ambient Term:
  vec4 Iamb = gl_FrontLightProduct[0].ambient;

  //calculate Diffuse Term:
  vec4 Idiff = gl_FrontLightProduct[0].diffuse * max(dot(N,L), 0.0);
  Idiff = clamp(Idiff, 0.0, 0.75);

  // calculate Specular Term:
  vec4 Ispec = gl_FrontLightProduct[0].specular
    * pow(max(dot(R,E),0.0),1.0);
  Ispec = clamp(Ispec, 0.0, 0.17);

  // write Total Color:
  gl_FragColor = gl_FrontLightModelProduct.sceneColor + Iamb + Idiff ;
  if (uShowSpecularHighlights)
    gl_FragColor += Ispec;

}


void main(void) {
  if (!uModernShader)
    {
      legacy_main();
      return;
    }

  vec3 ambientLightWeighting = uAmbientLightingColor;

  vec3 lightDirection = normalize(uPointLightingLocation - vPosition.xyz);
  vec3 normal = normalize(vTransformedNormal);

  vec3 specularLightWeighting = vec3(0.0, 0.0, 0.0);
  if (uShowSpecularHighlights) {
    vec3 eyeDirection = normalize(-vPosition.xyz);
    vec3 reflectionDirection = reflect(-lightDirection, normal);

    float specularLightBrightness = pow(max(dot(reflectionDirection, eyeDirection), 0.0), uMaterialShininess);
    specularLightWeighting = uPointLightingSpecularColor * specularLightBrightness;
  }

  float diffuseLightBrightness = max(dot(normal, lightDirection), 0.0);
  vec3 diffuseLightWeighting = uPointLightingDiffuseColor * diffuseLightBrightness;

  vec3 materialAmbientColor = uMaterialAmbientColor;
  vec3 materialDiffuseColor = uMaterialDiffuseColor;
  vec3 materialSpecularColor = uMaterialSpecularColor;
  vec3 materialEmissiveColor = uMaterialEmissiveColor;
  float alpha = 1.0;
  if (uUseTextures) {
    vec4 textureColor = texture2D(uSampler, vec2(vTextureCoord.s, vTextureCoord.t));
    materialAmbientColor = materialAmbientColor * textureColor.rgb;
    materialDiffuseColor = materialDiffuseColor * textureColor.rgb;
    materialEmissiveColor = materialEmissiveColor * textureColor.rgb;
    alpha = textureColor.a;
  }
  gl_FragColor = vec4(
                      materialAmbientColor * ambientLightWeighting
                      + materialDiffuseColor * diffuseLightWeighting
                      + materialSpecularColor * specularLightWeighting
                      + materialEmissiveColor,
                      alpha
                      );
}
