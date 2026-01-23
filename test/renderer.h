#pragma once
#include <glm/glm.hpp>

#include "scene.h"
#include "camera.h"

class Renderer {
public:
    Renderer();
    ~Renderer();

    void init();
    void draw(const Scene& scene, const Camera& cam);

    // helpers used da app for mouse / camera velocity stuffz
    void setPrevCameraPos(const glm::vec3& p);
    glm::vec3 getPrevCameraPos() const;

    void setLastMouse(float x, float y);
    float getLastX() const;
    float getLastY() const;

private:
    struct Impl;
    Impl* _p;
};
