#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

enum class ShapeType {
    None,
    Pyramid,
    Rect,
    Sphere
};

struct Shape {
 
    ShapeType type = ShapeType::None;
    int id = -1;

    // transf
    glm::vec3 pos = glm::vec3(0.0f);
    glm::vec3 scale = glm::vec3(1.0f);
    glm::quat rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);//rotacao

    // mov lin
    glm::vec3 vel = glm::vec3(0.0f);
    float mass = 1.0f;
    float invMass = 1.0f; //cache pra massa

    // mov rot
    glm::vec3 angularVel = glm::vec3(0.0f);
    float inertia = 1.0f;
    float invInertia = 1.0f; // cache inercia

    // flag de state ( estabilidade)
    bool isStatic = false;    // se true massa e iner sao  infinitas
    bool isDragging = false;  // se true  a fisica ignora gravidade

    // controle de gravidade
    bool useGravity = true;

    // slee[p wake
    bool isSleeping = false;
    float sleepTimer = 0.0f;
};

// massa e inercia
inline float getInvMass(const Shape& s) {
    if (s.isStatic || s.isDragging) return 0.0f;
    return (s.mass > 0.0f) ? 1.0f / s.mass : 0.0f;
}

inline float getInvInertia(const Shape& s) {
    if (s.isStatic || s.isDragging) return 0.0f;
    return (s.inertia > 0.0f) ? 1.0f / s.inertia : 0.0f;
}
struct Contact {
    Shape* a;
    Shape* b;
    glm::vec3 point;
    glm::vec3 normal;
    float penetration;
};

struct PyramidFace {
    glm::vec3 n;
    float d; // plano: dot(n, x) + d = 0
};




struct Selection {
    ShapeType type = ShapeType::None;
    int index = -1;
    float distance = 0.0f;
};

struct Ray {
    glm::vec3 o; // origin
    glm::vec3 d; // direction 
};
