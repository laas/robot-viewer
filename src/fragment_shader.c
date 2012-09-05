#ifdef GL_ES
precision highp float;
#endif

// global params
uniform float uLightSpecularBrightness;
uniform float uLightDiffuseBrightness;
uniform float uMaxSpecular;
uniform float uMaterialAmbientIntensity;


varying vec2 vTextureCoord;
varying vec3 vTransformedNormal;
varying vec4 vPosition;

uniform vec4 uMaterialAmbientColor;
uniform vec4 uMaterialDiffuseColor;
uniform vec4 uMaterialSpecularColor;
uniform vec4 uMaterialEmissiveColor;
uniform float uMaterialShininess;

uniform bool uShowSpecularHighlights;
uniform bool uUseTextures;

uniform vec4 uAmbientLightingColor;

uniform vec3 uPointLightingLocation;
uniform vec4 uPointLightingDiffuseColor;
uniform vec4 uPointLightingSpecularColor;

uniform sampler2D uSampler;

uniform bool uModernShader;



vec4 compute_color(vec4 materialAmbientColor,
                   vec4 materialDiffuseColor,
                   vec4 materialSpecularColor,
                   vec4 materialEmissiveColor,
                   float shininess,
                   vec4 ambientLightWeighting,
                   vec3 pointLightLocation,
                   vec4 pointLightDiffuseColor,
                   vec4 pointLightSpecularColor
                   )
{

  vec3 lightDirection = normalize(pointLightLocation - vPosition.xyz);
  vec3 normal = normalize(vTransformedNormal);

  vec4 specularLightWeighting = vec4(0.0, 0.0, 0.0, 0.0);
  if (uShowSpecularHighlights) {
    vec3 eyeDirection = normalize(-vPosition.xyz);
    vec3 reflectionDirection = reflect(-lightDirection, normal);
    float specularLightBrightness = pow(max(uLightSpecularBrightness*
                                            dot(reflectionDirection, eyeDirection), 0.0),
                                        shininess);
    specularLightWeighting = pointLightSpecularColor * specularLightBrightness;
  }

  float diffuseLightBrightness = max(uLightDiffuseBrightness*
                                     dot(normal, lightDirection), 0.0);
  vec4 diffuseLightWeighting = pointLightDiffuseColor * diffuseLightBrightness;

  float alpha = 1.0;
  if (uUseTextures) {
    vec4 textureColor = texture2D(uSampler, vec2(vTextureCoord.s, vTextureCoord.t));
    materialAmbientColor = materialAmbientColor * vec4(textureColor.rgb , 1.);
    materialDiffuseColor = materialDiffuseColor * vec4(textureColor.rgb, 1.);
    materialEmissiveColor = materialEmissiveColor * vec4(textureColor.rgb, 1.);
    alpha = textureColor.a;
  }
  vec4 Idiff, Ispec, Iamb, Iemis, res;
  Idiff =  materialDiffuseColor * diffuseLightWeighting ;
  Ispec = materialSpecularColor * specularLightWeighting;
  Ispec = clamp(Ispec, 0.0, uMaxSpecular);
  //Iamb = materialAmbientColor * ambientLightWeighting;
  Iamb = materialAmbientColor;
  Iemis = materialEmissiveColor;
  res = ( Iamb + Idiff + Iemis + Ispec);
  res = res + gl_Color;
  res = clamp(res, 0.0, 1.);
  return res;
}

void main(void) {
  if (uModernShader)
    {
      //vec4 ambientColor = uMaterialDiffuseColor*uMaterialAmbientIntensity;
      vec4 ambientColor = uAmbientLightingColor*0.1;

      gl_FragColor = compute_color( ambientColor,
                                    uMaterialDiffuseColor,
                                    uMaterialSpecularColor,
                                    uMaterialEmissiveColor,
                                    uMaterialShininess,
                                    uAmbientLightingColor,
                                    uPointLightingLocation,
                                    uPointLightingDiffuseColor,
                                    uPointLightingSpecularColor
                                    );
    }
  else
    {
      gl_FragColor = compute_color( gl_FrontMaterial.ambient,
                                    gl_FrontMaterial.diffuse,
                                    gl_FrontMaterial.specular,
                                    gl_FrontMaterial.emission,
                                    gl_FrontMaterial.shininess,
                                    gl_LightSource[0].ambient,
                                    gl_LightSource[0].position.xyz,
                                    gl_LightSource[0].diffuse,
                                    gl_LightSource[0].specular
                                    );
    }
  return;
}
