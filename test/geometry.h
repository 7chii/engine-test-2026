#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <array>



static constexpr int MAX_SHAPE_VERTS = 128;
static constexpr int MAX_SHAPE_FACES = 128;

struct Face
{
    glm::vec3 n;
    float d; // plano: dot(normal, x) + d = 0
};


struct GeometryData
{
    std::array<glm::vec3, MAX_SHAPE_VERTS> vertices;
    std::array<Face, MAX_SHAPE_FACES> faces;

    int vertexCount = 0;
    int faceCount = 0;
};