#pragma once
#include "scene.h"

// Update physics for the given scene using the provided delta time (seconds).
void updatePhysics(Scene& scene, float dt);
glm::mat3 computeLocalInertiaTensor(const Shape& s);
void allocateShapeGeometry(Shape& s); 
void setupRectShape(Shape& s);
void setupPyramidShape(Shape& s);
void setupSphereShape(Shape& s, float radius);
//static void setupCapsuleShape(Shape& s, float height);
//static void setupConvexHullShape(Shape& s, int vertCount, int faceCount)