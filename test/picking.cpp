#include "picking.h"
#include "camera.h"

#include <algorithm>
#include <cfloat>


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

Selection pickShape(const Scene& scene, const Camera& cam) {
    Selection sel;
    float closest = FLT_MAX;
    Ray ray = { cam.pos, glm::normalize(cam.front) };

    auto testList = [&](const std::vector<Shape>& list, ShapeType type) {
        for (int i = 0; i < (int)list.size(); ++i) {
            const Shape& s = list[i];
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

    testList(scene.pyramidsRef(), ShapeType::Pyramid);
    testList(scene.rectsRef(), ShapeType::Rect);
    testList(scene.spheresRef(), ShapeType::Sphere);

    return sel;
}