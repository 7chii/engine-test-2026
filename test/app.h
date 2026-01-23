#pragma once

#include <glm/glm.hpp>

struct GLFWwindow;

#include "scene.h"
#include "camera.h"
#include "renderer.h"

class App {
public:
    App();
    ~App();

    bool init();

    void frame();

    bool shouldClose() const;

    void shutdown();

private:
    GLFWwindow* _window = nullptr;

    // core subsystems
    Renderer _renderer;
    Scene _scene;
    Camera _camera;

    // time
    float _lastFrame = 0.0f;
    float _deltaTime = 0.0f;

    // dragging state
    bool _dragging = false;
    float _dragDistance = 0.0f;
    glm::vec3 _prevTargetPos = glm::vec3(0.0f);
    glm::vec3 _lastTargetPos = glm::vec3(0.0f);
    float _lastTargetDt = 0.0f;
    bool _havePrevTarget = false;
    bool _rightPrev = false;

    // camera state
    bool _cameraActive = true;

    // helpers
    void processInput();
    static void cursorCallback(GLFWwindow* window, double xpos, double ypos);
};