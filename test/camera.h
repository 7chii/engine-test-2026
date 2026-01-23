#pragma once
#include <glm/glm.hpp>

struct Camera {
    glm::vec3 pos{ 0,5,15 };
    glm::vec3 front{ 0,-0.2f,-1 };
    glm::vec3 up{ 0,1,0 };

    float yaw = -90.f;
    float pitch = 0.f;

    void updateMouse(float dx, float dy);
    void updateKeyboard(float dt);
    void onMouse(float dx, float dy);
};
