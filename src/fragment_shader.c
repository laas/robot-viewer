#ifdef GL_ES
precision highp float;
#endif

varying vec2 vTextureCoord;
varying vec3 vTransformedNormal;
varying vec4 vPosition;

uniform vec3 uMaterialAmbientColor;
uniform vec3 uMaterialDiffuseColor;
uniform vec3 uMaterialSpecularColor;
uniform vec3 uMaterialEmissiveColor;
uniform float uMaterialShininess;

uniform bool uShowSpecularHighlights;
uniform bool uUseTextures;

uniform vec3 uAmbientLightingColor;

uniform vec3 uPointLightingLocation;
uniform vec3 uPointLightingDiffuseColor;
uniform vec3 uPointLightingSpecularColor;

uniform sampler2D uSampler;

uniform bool uModernShader;

vec4 compute_color(vec3 materialAmbientColor,
                   vec3 materialDiffuseColor,
                   vec3 materialSpecularColor,
                   vec3 materialEmissiveColor,
                   float shininess,
                   vec3 ambientLightWeighting,
                   vec3 pointLightLocation,
                   vec3 pointLightDiffuseColor,
                   vec3 pointLightSpecularColor
                   )
{

  vec3 lightDirection = normalize(pointLightLocation - vPosition.xyz);
  vec3 normal = normalize(vTransformedNormal);

  vec3 specularLightWeighting = vec3(0.0, 0.0, 0.0);
  if (uShowSpecularHighlights) {
    vec3 eyeDirection = normalize(-vPosition.xyz);
    vec3 reflectionDirection = reflect(-lightDirection, normal);

    float specularLightBrightness = pow(max(dot(reflectionDirection, eyeDirection), 0.0),
                                        shininess);
    specularLightWeighting = pointLightSpecularColor * specularLightBrightness;
  }

  float diffuseLightBrightness = max(dot(normal, lightDirection), 0.0);
  vec3 diffuseLightWeighting = pointLightDiffuseColor * diffuseLightBrightness;

  float alpha = 1.0;
  if (uUseTextures) {
    vec4 textureColor = texture2D(uSampler, vec2(vTextureCoord.s, vTextureCoord.t));
    materialAmbientColor = materialAmbientColor * textureColor.rgb;
    materialDiffuseColor = materialDiffuseColor * textureColor.rgb;
    materialEmissiveColor = materialEmissiveColor * textureColor.rgb;
    alpha = textureColor.a;
  }
  vec4 Idiff, Ispec, Iamb, Iemis, res;
  Idiff =  vec4(materialDiffuseColor * diffuseLightWeighting, alpha) ;
  Idiff = clamp(Idiff, 0.0, 0.75);
  Ispec = vec4(materialSpecularColor * specularLightWeighting, alpha);
  Ispec = clamp(Ispec, 0.0, 0.05);
  Iamb = vec4(materialAmbientColor * ambientLightWeighting, 0.);
  Iemis = vec4(materialEmissiveColor, alpha);
  res = ( Iamb + Idiff + Iemis + Ispec);
  res = res + gl_Color;
  //res = clamp(res, 0.0, 1.);
  return res;
}

void main(void) {
  if (uModernShader)
    {
      gl_FragColor = compute_color( uMaterialAmbientColor,
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
                                    //uMaterialAmbientColor,
                                    gl_FrontMaterial.diffuse,
                                    //uMaterialDiffuseColor,
                                    gl_FrontMaterial.specular,
                                    //uMaterialSpecularColor,
                                    gl_FrontMaterial.emission,
                                    //uMaterialEmissiveColor,
                                    gl_FrontMaterial.shininess,
                                    //uMaterialShininess,
                                    gl_LightSource[0].ambient,
                                    //vec3(0.1,0.1,0.1),
                                    gl_LightSource[0].position.xyz,
                                    //vec3(5.0, 4.0 ,5.0),
                                    gl_LightSource[0].diffuse,
                                    //vec3(0.8,0.8,0.8),
                                    gl_LightSource[0].specular
                                    //vec3(0.8,0.8,0.8)
                                    );
    }
  return;
}
