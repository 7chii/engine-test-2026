#include <glad/glad.h>

#include "app.h"

#include <iostream>
#include <vector>
#include <cmath>

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "picking.h"
#include "physics.h"
#include "console.h"
#include "shape.h"
#include "scene.h"


// ------------------------------------------------------------
// Lista global de shapes para consultas de grounded/suporte
// ------------------------------------------------------------
std::vector<Shape*> gShapes;

// - centralizar init loop shutdown do monolito.
// - guarda ponteiro pra  ela mesma  na janelinha do GLFW  com o user pointer procallback do cursor
//   fazer a camera rodar.

App::App() = default;
App::~App() = default;

bool App::init() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);

    _window = glfwCreateWindow(1200, 800, "teste engine 2025 cena basica", nullptr, nullptr);
    if (!_window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    //  user pointer callback
    glfwSetWindowUserPointer(_window, this);
    glfwMakeContextCurrent(_window);

    // set cursor callback pra instancia da app
    glfwSetCursorPosCallback(_window, App::cursorCallback);
    // start with camera active e cursor desativado
    glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    _cameraActive = true;

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        glfwDestroyWindow(_window);
        glfwTerminate();
        return false;
    }

    glViewport(0, 0, 1200, 800);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);

    // imgui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(_window, true);
    ImGui_ImplOpenGL3_Init("#version 460");

    // renderer init ( shaders and VAOs / sphere buffers)
    _renderer.init();

    _scene.pyramidsRef().clear();
    _scene.rectsRef().clear();
    _scene.spheresRef().clear();

    Shape pyramid;
    pyramid.type = ShapeType::Pyramid;
    //pyramid.id = (int)_scene.pyramidsRef().size();
    pyramid.id = 3;
    pyramid.scale = glm::vec3(1.0f);
    pyramid.pos = glm::vec3(-3.0, 0.51, 0.0f);
    pyramid.mass = 1.0f;
    pyramid.requestedFaceCount = 5;
    pyramid.requestedVertexCount = 5;
	setupPyramidShape(pyramid);
    pyramid.inertia = computeLocalInertiaTensor(pyramid);
    pyramid.invInertia = glm::inverse(pyramid.inertia);
    _scene.pyramidsRef().push_back(pyramid);

    Shape rect;
    rect.type = ShapeType::Rect;
    //rect.id = (int)_scene.rectsRef().size();
    rect.id = 2;
    rect.pos = glm::vec3(0.0f, 3.0f, 0.0f);
    rect.scale = glm::vec3(2.0f, 1.0f, 1.0f);
    rect.mass = 3.0f;
    rect.requestedFaceCount = 6;
	rect.requestedVertexCount = 8;
    setupRectShape(rect);
    rect.inertia = computeLocalInertiaTensor(rect);
    rect.invInertia = glm::inverse(rect.inertia);
    _scene.rectsRef().push_back(rect);

    Shape sphere;
    sphere.type = ShapeType::Sphere;
    //sphere.id = (int)_scene.spheresRef().size();
    sphere.id = 1;
    sphere.scale = glm::vec3(1.0f);
    sphere.radius = 1.0f;
    sphere.pos = glm::vec3(3.0, 4.0, 1.0f);
    sphere.mass = 2.0f;
    setupSphereShape(sphere, sphere.radius);
    sphere.inertia = computeLocalInertiaTensor(sphere);
    sphere.invInertia = glm::inverse(sphere.inertia);
    _scene.spheresRef().push_back(sphere);
	
    
    
    return true;
}

void App::frame() {
    // tempo
    float now = (float)glfwGetTime();
    _deltaTime = now - _lastFrame;
    _lastFrame = now;

    // camera velocity (pra quando sortar objeto uai)
    glm::vec3 cameraVel = (_deltaTime > 1e-6f) ? (_camera.pos - _renderer.getPrevCameraPos()) / _deltaTime : glm::vec3(0.0f);

    processInput();

    // imgui new frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // console ui
    drawConsole(_scene);
    bool rightNow = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
	// iniciar drag
    if (rightNow && !_rightPrev) {
        Selection sel = pickShape(_scene, _camera);
        _scene.selectionRef() = sel;

        if (sel.type != ShapeType::None) {
            _dragging = true;
            _dragDistance = sel.distance;

            glm::vec3 hit =
                _camera.pos + _camera.front * _dragDistance;

            _prevTargetPos = hit;
            _lastTargetPos = hit;
            _lastTargetDt = 1e-6f;
            _havePrevTarget = false;
        }
        else {
            _dragging = false;
        }
    }

    

    auto getShapePtrFromScene = [&](const Selection& sel) -> Shape* {
        if (sel.type == ShapeType::Pyramid) {
            auto& vec = _scene.pyramidsRef();
            if (sel.index >= 0 && sel.index < (int)vec.size()) return &vec[sel.index];
        }
        else if (sel.type == ShapeType::Rect) {
            auto& vec = _scene.rectsRef();
            if (sel.index >= 0 && sel.index < (int)vec.size()) return &vec[sel.index];
        }
        else if (sel.type == ShapeType::Sphere) {
            auto& vec = _scene.spheresRef();
            if (sel.index >= 0 && sel.index < (int)vec.size()) return &vec[sel.index];
        }
        return nullptr;
        };

    // crosshair
    /*
    * if (!rightNow && _rightPrev) {
        if (_scene.selectionRef().type != ShapeType::None && _havePrevTarget) {
            Shape* s = getShapePtrFromScene(_scene.selectionRef());
            if (s) {
                glm::vec3 throwVel =
                    (_lastTargetPos - _prevTargetPos) /
                    std::max(_lastTargetDt, 1e-6f);

                throwVel += cameraVel * 0.6f;

                float vmax = 50.0f;
                if (glm::length(throwVel) > vmax)
                    throwVel = glm::normalize(throwVel) * vmax;

                s->vel = throwVel;
                s->isDragging = false;
                s->isSleeping = false;
                s->sleepTimer = 0.0f;
            }
        }

        _dragging = false;
        _scene.selectionRef() = Selection{};
    }

    */
    
    // helper  return a pointer into the scene storage for a select
   
    if (_dragging && _scene.selectionRef().type != ShapeType::None) {
        glm::vec3 currentTarget = _camera.pos + _camera.front * _dragDistance;

        _prevTargetPos = _lastTargetPos;
        _lastTargetPos = currentTarget;
        _lastTargetDt = std::max(_deltaTime, 1e-6f);
        _havePrevTarget = true;

        Shape* s = getShapePtrFromScene(_scene.selectionRef());
        if (s) {
            s->pos = currentTarget;
            s->vel = glm::vec3(0.0f);
            s->angularVel = glm::vec3(0.0f);
            s->useGravity = true;
            s->isDragging = true;
            s->isSleeping = false;
            s->sleepTimer = 0.0f;
        }
    }
    /*
    else if (!_dragging && _scene.selectionRef().type != ShapeType::None) {
        Shape* s = getShapePtrFromScene(_scene.selectionRef());
        if (s) {
            s->isDragging = false;
            s->isSleeping = false;
			s->groundedStableTimer = 0.0f;
            s->sleepTimer = 0.0f;
        }
    }
    */
    

    if (!rightNow && _rightPrev) {
        // release throw
        if (_scene.selectionRef().type != ShapeType::None && _havePrevTarget) {
            Selection cur = _scene.selectionRef();
            Shape* s = nullptr;
            if (cur.type == ShapeType::Pyramid) {
                auto& v = _scene.pyramidsRef();
                if (cur.index >= 0 && cur.index < (int)v.size()) s = &v[cur.index];
            }
            else if (cur.type == ShapeType::Rect) {
                auto& v = _scene.rectsRef();
                if (cur.index >= 0 && cur.index < (int)v.size()) s = &v[cur.index];
            }
            else if (cur.type == ShapeType::Sphere) {
                auto& v = _scene.spheresRef();
                if (cur.index >= 0 && cur.index < (int)v.size()) s = &v[cur.index];
            }
            if (s) {

                glm::vec3 throwVel = (_lastTargetPos - _prevTargetPos) / std::max(_lastTargetDt, 1e-6f);
                throwVel += cameraVel * 0.6f;
                float vmax = 50.0f;
                if (glm::length(throwVel) > vmax) throwVel = glm::normalize(throwVel) * vmax;
                s->vel = throwVel;
                s->angularVel = glm::vec3(0.0f);
                s->isDragging = false;
                s->isSleeping = false;
                s->sleepTimer = 0.0f;
            }
        }
        _dragging = false;
    }
    _rightPrev = rightNow;


    // physics 
    updatePhysics(_scene, _deltaTime);



    // render
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    _renderer.draw(_scene, _camera);

    // render imgui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(_window);
    glfwPollEvents();

    // update renderer ultima pos pra camera pro calculo da vel do  proximo frame
    _renderer.setPrevCameraPos(_camera.pos);
}

bool App::shouldClose() const {
    return !_window || glfwWindowShouldClose(_window);
}

void App::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (_window) glfwDestroyWindow(_window);
    glfwTerminate();
}

//  cursor callback :dispatch to instance
void App::cursorCallback(GLFWwindow* window, double xpos, double ypos) {
    App* self = static_cast<App*>(glfwGetWindowUserPointer(window));
    if (!self) return;
    // forward to camera only if the camera is active
    if (!self->_cameraActive) return;
    self->_camera.onMouse((float)(xpos - self->_renderer.getLastX()), (float)(self->_renderer.getLastY() - ypos));
    // update renderer last mouse pos
    self->_renderer.setLastMouse((float)xpos, (float)ypos);
}

// private helpers
void App::processInput() {
    if (glfwGetKey(_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(_window, true);

    // tab toggles camera mode and cursor
    static bool tabPressed = false;
    if (glfwGetKey(_window, GLFW_KEY_TAB) == GLFW_PRESS && !tabPressed) {
        // toggle camera active
        _cameraActive = !_cameraActive;
        // update cursor mode 
        glfwSetInputMode(_window, GLFW_CURSOR, _cameraActive ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        tabPressed = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_TAB) == GLFW_RELEASE) {
        tabPressed = false;
    }

    
    if (!_cameraActive) return;

    float speed = 5.0f * _deltaTime;
    if (glfwGetKey(_window, GLFW_KEY_UP) == GLFW_PRESS) _camera.pos += speed * _camera.front;
    if (glfwGetKey(_window, GLFW_KEY_DOWN) == GLFW_PRESS) _camera.pos -= speed * _camera.front;
    if (glfwGetKey(_window, GLFW_KEY_LEFT) == GLFW_PRESS) _camera.pos -= glm::normalize(glm::cross(_camera.front, _camera.up)) * speed;
    if (glfwGetKey(_window, GLFW_KEY_RIGHT) == GLFW_PRESS) _camera.pos += glm::normalize(glm::cross(_camera.front, _camera.up)) * speed;
}
