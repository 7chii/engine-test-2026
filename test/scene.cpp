#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>

#include <glad/glad.h>
#include "scene.h"


std::vector<Shape> pyramids;
std::vector<Shape> rects;
std::vector<Shape> spheres;

Selection currentSelection;


Ray getCameraRay(const glm::vec3& camPos, const glm::vec3& camFront) {
    return { camPos, glm::normalize(camFront) };
}

static bool rayAABB(const Ray& r, const glm::vec3& mn, const glm::vec3& mx, float& tHit) {
    float tmin = (mn.x - r.o.x) / (fabs(r.d.x) < 1e-8f ? 1e-8f : r.d.x);
    float tmax = (mx.x - r.o.x) / (fabs(r.d.x) < 1e-8f ? 1e-8f : r.d.x);
    if (tmin > tmax) std::swap(tmin, tmax);

    float tymin = (mn.y - r.o.y) / (fabs(r.d.y) < 1e-8f ? 1e-8f : r.d.y);
    float tymax = (mx.y - r.o.y) / (fabs(r.d.y) < 1e-8f ? 1e-8f : r.d.y);
    if (tymin > tymax) std::swap(tymin, tymax);

    if (tmin > tymax || tymin > tmax) return false;
    if (tymin > tmin) tmin = tymin;

    float tzmin = (mn.z - r.o.z) / (fabs(r.d.z) < 1e-8f ? 1e-8f : r.d.z);
    float tzmax = (mx.z - r.o.z) / (fabs(r.d.z) < 1e-8f ? 1e-8f : r.d.z);
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if (tmin > tzmax || tzmin > tmax) return false;
    if (tzmin > tmin) tmin = tzmin;

    tHit = tmin;
    return true;
}


Selection pickShape(const glm::vec3& camPos, const glm::vec3& camFront) {
    Selection sel;
    float closest = FLT_MAX;
    Ray ray = getCameraRay(camPos, camFront);

    auto testList = [&](std::vector<Shape>& list, ShapeType type) {
        for (int i = 0; i < (int)list.size(); ++i) {
            Shape& s = list[i];
            glm::vec3 half = s.scale * 0.5f;
            glm::vec3 mn = s.pos - half;
            glm::vec3 mx = s.pos + half;

            float t;
            if (rayAABB(ray, mn, mx, t)) {
                if (t > 0 && t < closest) {
                    closest = t;
                    sel = { type, i, t };
                }
            }
        }
    };

    testList(pyramids, ShapeType::Pyramid);
    testList(rects, ShapeType::Rect);
    testList(spheres, ShapeType::Sphere);

    return sel;
}

Shape* getShapeRef(ShapeType t, int idx) {
    if (t == ShapeType::Pyramid && idx >= 0 && idx < (int)pyramids.size()) return &pyramids[idx];
    if (t == ShapeType::Rect && idx >= 0 && idx < (int)rects.size()) return &rects[idx];
    if (t == ShapeType::Sphere && idx >= 0 && idx < (int)spheres.size()) return &spheres[idx];
    return nullptr;
}

static float invMass(const Shape& s) {
    return s.mass > 0.0f ? 1.0f / s.mass : 0.0f;
}


void updatePhysics(float dt) {
    if (dt <= 0.0f) return;

    const glm::vec3 gravity(0, -9.81f, 0);

    auto integrate = [&](Shape& s) {
        s.vel += gravity * dt;
        s.pos += s.vel * dt;
    };

    for (auto& s : pyramids) integrate(s);
    for (auto& s : rects)    integrate(s);
    for (auto& s : spheres)  integrate(s);

    // ground
    auto ground = [&](Shape& s) {
        float halfY = s.scale.y * 0.5f;
        if (s.pos.y - halfY < 0.0f) {
            s.pos.y = halfY;
            if (s.vel.y < 0) s.vel.y = 0;
            s.vel.x *= 0.98f;
            s.vel.z *= 0.98f;
        }
    };

    for (auto& s : pyramids) ground(s);
    for (auto& s : rects)    ground(s);
    for (auto& s : spheres)  ground(s);
}


void drawScene(
    const Scene& scene,
    unsigned int shaderProgram,
    unsigned int vaoPyramid,
    unsigned int vaoRect,
    unsigned int vaoSphere,
    int sphereIndexCount,
    const glm::mat4& view,
    const glm::mat4& projection
) {
    glUseProgram(shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    const Selection& sel = scene.selectionRef();

    auto drawWithOutline = [&](unsigned int vao, const Shape& s, int count, bool indexed, ShapeType t, int idx) {
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);

        glm::mat4 m = glm::translate(glm::mat4(1.0f), s.pos);
        m = glm::scale(m, s.scale);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(m));
        glUniform3f(glGetUniformLocation(shaderProgram, "color"), 0.7f, 0.7f, 0.9f);

        glBindVertexArray(vao);
        if (indexed) glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, 0);
        else glDrawArrays(GL_TRIANGLES, 0, count);

        if (sel.type == t && sel.index == idx) {
            glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
            glStencilMask(0x00);
            glDisable(GL_DEPTH_TEST);

            glm::mat4 om = glm::scale(m, glm::vec3(1.05f));
            glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(om));
            glUniform3f(glGetUniformLocation(shaderProgram, "color"), 1, 0, 0);

            if (indexed) glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, 0);
            else glDrawArrays(GL_TRIANGLES, 0, count);

            glEnable(GL_DEPTH_TEST);
            glStencilMask(0xFF);
            glStencilFunc(GL_ALWAYS, 0, 0xFF);
        }
    };

    const auto& ps = scene.pyramidsRef();
    const auto& rs = scene.rectsRef();
    const auto& ss = scene.spheresRef();

    for (int i = 0; i < (int)ps.size(); ++i)
        drawWithOutline(vaoPyramid, ps[i], 12, false, ShapeType::Pyramid, i);

    for (int i = 0; i < (int)rs.size(); ++i)
        drawWithOutline(vaoRect, rs[i], 36, false, ShapeType::Rect, i);

    for (int i = 0; i < (int)ss.size(); ++i)
        drawWithOutline(vaoSphere, ss[i], sphereIndexCount, true, ShapeType::Sphere, i);
}
