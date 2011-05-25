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


    void main(void) {
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
