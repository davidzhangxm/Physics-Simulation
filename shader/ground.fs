#version 330 core

in vec2 TexCoord;
in vec3 Normal;
in vec3 FragPos;

out vec4 FragColor;

uniform sampler2D texture1;

uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 viewPos;

// 255, 251, 240
// rgb(235, 225, 176)
void main(){
    //FragColor = vec4(1.0f, 0.98f, 0.94f, 1.0f);
    //FragColor = vec4(0.92f, 0.88f, 0.69f, 1.0f);
    vec3 planeColor = vec3(0.92f, 0.88f, 0.69f);

    //FragColor = texture(texture1, TexCoord);

    // ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;
    // diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;

    vec3 result = (ambient + diffuse + specular) * planeColor;
    FragColor = vec4(result, 1.0);

}