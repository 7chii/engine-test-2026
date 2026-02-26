#include <algorithm>
#include "shape.h"

//----------------------------------------------------------
// ALLOCATOR
//----------------------------------------------------------

void allocateShapeGeometry(Shape& s)
{
    // Clamp seguro estilo console
    s.geometry.vertexCount = std::clamp(
        s.requestedVertexCount,
        0,
        MAX_SHAPE_VERTS
    );

    s.geometry.faceCount = std::clamp(
        s.requestedFaceCount,
        0,
        MAX_SHAPE_FACES
    );

    // Marca para rebuild de world data
    s.geometryDirty = true;
}
void setupRectShape(Shape& s)
{
    s.type = ShapeType::Rect;

    s.requestedVertexCount = 8;
    s.requestedFaceCount = 6;

    allocateShapeGeometry(s);

    s.radius = 0.0f;
    s.halfHeight = 0.0f;
}
void setupPyramidShape(Shape& s)
{
    s.type = ShapeType::Pyramid;

    s.requestedVertexCount = 5;
    s.requestedFaceCount = 5;

    allocateShapeGeometry(s);

    s.radius = 0.0f;
    s.halfHeight = 0.0f;
}
void setupSphereShape(Shape& s, float radius)
{
    s.type = ShapeType::Sphere;

    s.radius = std::max(0.0f, radius);
    s.halfHeight = 0.0f;

    s.requestedVertexCount = 0;
    s.requestedFaceCount = 0;

    allocateShapeGeometry(s);
}
void setupCapsuleShape(Shape& s, float radius, float height)
{
    s.type = ShapeType::Capsule;

    s.radius = std::max(0.0f, radius);
    s.halfHeight = std::max(0.0f, height * 0.5f);

    constexpr int segments = 16;

    s.requestedVertexCount = segments * 2;
    s.requestedFaceCount = segments;

    allocateShapeGeometry(s);
}
void setupConvexHullShape(Shape& s, int vertCount, int faceCount)
{
    s.type = ShapeType::ConvexHull;

    s.requestedVertexCount = std::max(0, vertCount);
    s.requestedFaceCount = std::max(0, faceCount);

    allocateShapeGeometry(s);

    s.radius = 0.0f;
    s.halfHeight = 0.0f;
}