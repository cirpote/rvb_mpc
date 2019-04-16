// shader associated with AssimpLoader

attribute   vec3 vertexPosition;
attribute   vec2 vertexUV;
varying     vec2 textureCoords;
uniform     mat4 mvpMat;

void main()
{
    gl_Position     = mvpMat * vec4(vertexPosition, 1.0);
    textureCoords   = vertexUV;
}
