#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "shape.h"

// scene stores shape containers and state de selecao de   renderer / picking/gui
class Scene {
public:
    // acessores
    std::vector<Shape>& pyramidsRef() { return _pyramids; }
    std::vector<Shape>& rectsRef() { return _rects; }
    std::vector<Shape>& spheresRef() { return _spheres; }
    const std::vector<Shape>& pyramidsRef() const { return _pyramids; }
    const std::vector<Shape>& rectsRef() const { return _rects; }
    const std::vector<Shape>& spheresRef() const { return _spheres; }

    // selection stored with the cena pro renderer/picking/gui funfem
    Selection& selectionRef() { return _selection; }
    const Selection& selectionRef() const { return _selection; }

private:
    std::vector<Shape> _pyramids;
    std::vector<Shape> _rects;
    std::vector<Shape> _spheres;

    Selection _selection;
};


extern std::vector<Shape> pyramids;
extern std::vector<Shape> rects;
extern std::vector<Shape> spheres;
extern Selection currentSelection;

Ray getCameraRay(const glm::vec3& camPos, const glm::vec3& camFront);
Selection pickShape(const glm::vec3& camPos, const glm::vec3& camFront);
Shape* getShapeRef(ShapeType t, int idx);
void updatePhysics(float dt);

void drawScene(
    const Scene& scene,
    unsigned int shaderProgram,
    unsigned int vaoPyramid,
    unsigned int vaoRect,
    unsigned int vaoSphere,
    int sphereIndexCount,
    const glm::mat4& view,
    const glm::mat4& projection
);

void drawSceneWithShadow(
    const Scene& scene,
    unsigned int shadowShaderProgram,
    unsigned int vaoPyramid,
    unsigned int vaoRect,
    unsigned int vaoSphere,
    int sphereIndexCount,
    const glm::mat4& view,
    const glm::mat4& projection
);
