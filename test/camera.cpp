#include "camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>


void Camera::updateMouse(float dx, float dy) {
    const float sensitivity = 0.05f;
    yaw += dx * sensitivity;
    pitch += dy * sensitivity;
    pitch = std::clamp(pitch, -89.0f, 89.0f);
    front = glm::normalize(glm::vec3(
        cos(glm::radians(yaw)) * cos(glm::radians(pitch)),
        sin(glm::radians(pitch)),
        sin(glm::radians(yaw)) * cos(glm::radians(pitch))
    ));
}

void Camera::onMouse(float dx, float dy) {
    updateMouse(dx, dy);
}

void Camera::updateKeyboard(float /*dt*/) {
    // blablaa
}

