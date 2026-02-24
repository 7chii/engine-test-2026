#pragma once
#include "scene.h"

// Update physics for the given scene using the provided delta time (seconds).
void updatePhysics(Scene& scene, float dt);
glm::mat3 computeLocalInertiaTensor(const Shape& s);