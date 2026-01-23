#include "physics.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cfloat>



static const glm::vec3 GRAVITY(0.0f, -9.81f, 0.0f);

static const float restitutionGround = 0.2f;
static const float frictionGround = 0.8f;

static const float sleepLinearThreshold = 0.03f;
static const float sleepAngularThreshold = 0.03f;
static const float sleepTimeThreshold = 0.6f;



static void wake(Shape& s) {
    if (s.isStatic) return;
    s.isSleeping = false;
    s.sleepTimer = 0.0f;
}



static float sphereRadius(const Shape& s) {
    return 0.5f * std::max({ s.scale.x, s.scale.y, s.scale.z });
}

static glm::vec3 getHalfExtents(const Shape& s) {
    return s.scale * 0.5f;
}


static glm::vec3 getLowestPointOnBox(const Shape& s) {
    glm::mat3 R = glm::mat3_cast(s.rot);
    glm::vec3 he = s.scale * 0.5f;

    glm::vec3 lowest = s.pos;
    float minY = FLT_MAX;

    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2) {
                glm::vec3 local(x * he.x, y * he.y, z * he.z);
                glm::vec3 world = s.pos + R * local;
                if (world.y < minY) {
                    minY = world.y;
                    lowest = world;
                }
            }
    return lowest;
}

static void getPyramidVertices(const Shape& s, std::vector<glm::vec3>& out) {
    out.clear();
    glm::mat3 R = glm::mat3_cast(s.rot);
    glm::vec3 he = s.scale * 0.5f;

    out.push_back(s.pos + R * glm::vec3(-he.x, -he.y, -he.z));
    out.push_back(s.pos + R * glm::vec3(he.x, -he.y, -he.z));
    out.push_back(s.pos + R * glm::vec3(he.x, -he.y, he.z));
    out.push_back(s.pos + R * glm::vec3(-he.x, -he.y, he.z));
    out.push_back(s.pos + R * glm::vec3(0.0f, he.y, 0.0f));
}static void getPyramidFaces(const Shape& p, std::vector<PyramidFace>& faces) {
    faces.clear();

    glm::mat3 R = glm::mat3_cast(p.rot);
    glm::vec3 he = p.scale * 0.5f;

    glm::vec3 base[4] = {
        p.pos + R * glm::vec3(-he.x, -he.y, -he.z),
        p.pos + R * glm::vec3(he.x, -he.y, -he.z),
        p.pos + R * glm::vec3(he.x, -he.y,  he.z),
        p.pos + R * glm::vec3(-he.x, -he.y,  he.z)
    };

    glm::vec3 apex = p.pos + R * glm::vec3(0, he.y, 0);

    for (int i = 0; i < 4; ++i) {
        glm::vec3 a = base[i];
        glm::vec3 b = base[(i + 1) % 4];
        glm::vec3 c = apex;

        glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));

        // garante que aponta para fora
        if (glm::dot(n, p.pos - a) > 0.0f)
            n = -n;

        float d = -glm::dot(n, a);
        faces.push_back({ n, d });
    }

    // base
    glm::vec3 nBase = glm::vec3(0, -1, 0);
    nBase = glm::normalize(R * nBase);

    float dBase = -glm::dot(nBase, base[0]);
    faces.push_back({ nBase, dBase });
}



static void getPyramidContactPoints(const Shape& s, std::vector<glm::vec3>& out) {
    std::vector<glm::vec3> verts;
    getPyramidVertices(s, verts);

    out.clear();
    for (auto& v : verts)
        if (v.y <= 0.002f)
            out.push_back(v);


}

// integracao

static void integrateVelocity(Shape& s, float dt) {
    if (s.isStatic || s.isDragging || s.isSleeping) return;

    if (s.useGravity)
        s.vel += GRAVITY * dt;

    s.vel *= std::exp(-0.05f * dt);
    s.angularVel *= std::exp(-0.15f * dt);
}

static void integratePosition(Shape& s, float dt) {
    if (s.isStatic || s.isDragging || s.isSleeping) return;
    s.pos += s.vel * dt;
}

static void integrateRotation(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging || s.isSleeping) return;
    if (getInvInertia(s) <= 0.0f) return;

    
    
    constexpr float ANGULAR_DAMPING = 0.98f; 
    s.angularVel *= ANGULAR_DAMPING;

    float w2 = glm::length2(s.angularVel);

   
    constexpr float ANGULAR_EPS2 = 1e-8f;
    if (w2 < ANGULAR_EPS2) {
        s.angularVel = glm::vec3(0.0f);
        return;
    }

    float w = std::sqrt(w2);
    glm::vec3 axis = s.angularVel / w;

    
    s.rot = glm::normalize(
        glm::angleAxis(w * dt, axis) * s.rot
    );
}


//IMPULSO

static void applyImpulse(Shape& s, const glm::vec3& J, const glm::vec3& cp) {
    if (s.isStatic) return;
    wake(s);

    s.vel += J * getInvMass(s);
    s.angularVel += glm::cross(cp - s.pos, J) * getInvInertia(s);
}

// CHAO

static void solveGroundContact(Shape& s, const glm::vec3& cp, float dt) {
    glm::vec3 n(0, 1, 0);

    float penetration = -cp.y;

    // SEM contato : sai imediatamente
    if (penetration <= 0.0f)
        return;
    wake(s);

    const float slop = 0.001f;          // tolerancia
    const float percent = 0.8f;         // correcao

    float correction = std::max(penetration - slop, 0.0f) * percent;
    s.pos.y += correction;


    glm::vec3 r = cp - s.pos;
    glm::vec3 vcp = s.vel + glm::cross(s.angularVel, r);
    float vn = glm::dot(vcp, n);

    float invM = getInvMass(s);
    float invI = getInvInertia(s);

    float jn = 0.0f;
    if (vn < 0.0f) {
        float denom = invM + invI * glm::length2(glm::cross(r, n));
        jn = -(1.0f + restitutionGround) * vn / (denom + 1e-9f);
        applyImpulse(s, jn * n, cp);
    }


    glm::vec3 vt = vcp - glm::dot(vcp, n) * n;
    float vtLen = glm::length(vt);
    if (vtLen > 1e-6f) {
        glm::vec3 t = -vt / vtLen;
        float jt = std::min(vtLen * s.mass, frictionGround * jn);
        applyImpulse(s, jt * t, cp);
    }

    if (glm::length(s.angularVel) < 0.15f)
        s.angularVel *= 0.2f;

    s.vel *= std::exp(-1.5f * dt);
}

static void handleGroundContact(Shape& s, float dt) {
    if (s.isStatic || s.isDragging) return;

    if (s.type == ShapeType::Sphere) {
        float r = sphereRadius(s) + 0.45;

        // ponto mais baixo da esfera
        glm::vec3 cp = s.pos - glm::vec3(0, r, 0);

        // velocidade do ponto de contato
        float vDown = s.vel.y;

        // contato proximo (sweep simples)
        bool touchingNow =
            (cp.y <= 0.001f) ||
            (cp.y > 0.0f && cp.y + vDown * dt <= 0.0f);

        if (touchingNow) {
            solveGroundContact(s, cp, dt);
        }
    }

    else if (s.type == ShapeType::Pyramid) {
        std::vector<glm::vec3> cps;
        getPyramidContactPoints(s, cps);

        // escolhe APENAS o ponto mais PENETRADO
        glm::vec3 deepest;
        float maxPenetration = 0.0f;
        bool hasContact = false;

        for (auto& cp : cps) {
            float pen = -cp.y;
            if (pen > maxPenetration) {
                maxPenetration = pen;
                deepest = cp;
                hasContact = true;
            }
        }

        if (hasContact) {
            solveGroundContact(s, deepest, dt);
        }
    }


    else {
        solveGroundContact(s, getLowestPointOnBox(s), dt);
    }
}
static void solveContact(Contact& c, float dt) {
    Shape& A = *c.a;
    Shape& B = *c.b;

    if (A.isStatic && B.isStatic) return;

    wake(A);
    wake(B);

    // normar
    glm::vec3 n = glm::normalize(c.normal);
    if (glm::length2(n) < 1e-8f) return;

    float invMA = getInvMass(A);
    float invMB = getInvMass(B);
    float invIA = getInvInertia(A);
    float invIB = getInvInertia(B);

    // vetor de contato
    glm::vec3 rA = c.point - A.pos;
    glm::vec3 rB = c.point - B.pos;

    // vel no contato
    glm::vec3 vA = A.vel + glm::cross(A.angularVel, rA);
    glm::vec3 vB = B.vel + glm::cross(B.angularVel, rB);
    glm::vec3 rv = vB - vA;

    float vn = glm::dot(rv, n);

    // correcao pos
    {
        const float slop = 0.001f;
        const float percent = 0.7f;

        float pen = std::max(c.penetration - slop, 0.0f);
        float denom = invMA + invMB;
        if (denom > 0.0f) {
            glm::vec3 corr = (pen * percent / denom) * n;
            if (!A.isStatic) A.pos -= corr * invMA;
            if (!B.isStatic) B.pos += corr * invMB;
        }
    }


    if (vn > 0.0f)
        return;

    // impulso normar
    float e = restitutionGround;

    float denomN =
        invMA + invMB +
        invIA * glm::length2(glm::cross(rA, n)) +
        invIB * glm::length2(glm::cross(rB, n));

    if (denomN <= 0.0f) return;

    float jn = -(1.0f + e) * vn / denomN;
    glm::vec3 impulseN = jn * n;

    if (!A.isStatic) {
        A.vel -= impulseN * invMA;
        A.angularVel -= glm::cross(rA, impulseN) * invIA;
    }
    if (!B.isStatic) {
        B.vel += impulseN * invMB;
        B.angularVel += glm::cross(rB, impulseN) * invIB;
    }

    //friccao
    rv = (B.vel + glm::cross(B.angularVel, rB)) -
        (A.vel + glm::cross(A.angularVel, rA));

    glm::vec3 t = rv - glm::dot(rv, n) * n;
    float tLen = glm::length(t);
    if (tLen < 1e-6f) return;

    t /= tLen;

    float denomT =
        invMA + invMB +
        invIA * glm::length2(glm::cross(rA, t)) +
        invIB * glm::length2(glm::cross(rB, t));

    if (denomT <= 0.0f) return;

    float jt = -glm::dot(rv, t) / denomT;

    float maxFriction = frictionGround * jn;
    jt = glm::clamp(jt, -maxFriction, maxFriction);

    glm::vec3 impulseT = jt * t;

    if (!A.isStatic) {
        A.vel -= impulseT * invMA;
        A.angularVel -= glm::cross(rA, impulseT) * invIA;
    }
    if (!B.isStatic) {
        B.vel += impulseT * invMB;
        B.angularVel += glm::cross(rB, impulseT) * invIB;
    }
}

static bool sphereSphere(Shape& a, Shape& b, Contact& c) {
    float ra = sphereRadius(a);
    float rb = sphereRadius(b);

    glm::vec3 d = b.pos - a.pos;
    float dist2 = glm::length2(d);
    float r = ra + rb;

    if (dist2 > r * r) return false;

    float dist = std::sqrt(dist2);
    glm::vec3 n = (dist > 1e-6f) ? d / dist : glm::vec3(0, 1, 0);

    c = { &a, &b, a.pos + n * ra, n, r - dist };
    return true;
}
static bool spherePyramid(Shape& s, Shape& p, Contact& c) {
    float r = sphereRadius(s);

    std::vector<PyramidFace> faces;
    getPyramidFaces(p, faces);

    float minPen = FLT_MAX;
    glm::vec3 bestN;

    for (auto& f : faces) {
        float dist = glm::dot(f.n, s.pos) + f.d;

        if (dist > (r + 0.4))
            return false;

        float pen = r - dist;
        if (pen < minPen) {
            minPen = pen;
            bestN = f.n;
        }
    }

    c.a = &p;
    c.b = &s;
    c.normal = bestN;
    c.penetration = minPen;
    c.point = s.pos - bestN * r;
    return true;
}
static bool boxPyramid(Shape& box, Shape& pyr, Contact& c)
{
    constexpr float GAP = 0.1f;

    //box
    glm::mat3 Rb = glm::mat3_cast(box.rot);
    glm::vec3 hb = box.scale * 0.5f;
    glm::vec3 boxUp = glm::normalize(Rb[1]);

    // vert
    std::vector<glm::vec3> boxVerts;
    boxVerts.reserve(8);
    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2)
                boxVerts.push_back(
                    box.pos + Rb * glm::vec3(x * hb.x, y * hb.y, z * hb.z)
                );

    
    std::vector<glm::vec3> pyrVerts;
    getPyramidVertices(pyr, pyrVerts);
    if (pyrVerts.size() < 5) return false;

    std::vector<PyramidFace> pyrFaces;
    getPyramidFaces(pyr, pyrFaces);

   
    {
        glm::vec3 e0 = pyrVerts[1] - pyrVerts[0];
        glm::vec3 e1 = pyrVerts[2] - pyrVerts[0];
        glm::vec3 baseN = glm::normalize(glm::cross(e0, e1));

        // base quase horizontal
        if (std::abs(glm::dot(baseN, boxUp)) > 0.85f)
        {
            glm::vec3 baseCentroid(0.0f);
            float baseMinProj = FLT_MAX;

            for (int i = 0; i < 4; ++i) {
                baseCentroid += pyrVerts[i];
                baseMinProj = std::min(baseMinProj, glm::dot(boxUp, pyrVerts[i]));
            }
            baseCentroid *= 0.25f;

            float boxTop = glm::dot(boxUp, box.pos) + hb.y;
            float penetration = (boxTop - baseMinProj) - GAP;

            if (penetration > 0.0f)
            {
                glm::vec3 local = glm::transpose(Rb) * (baseCentroid - box.pos);
                const float margin = 0.01f;

                if (std::abs(local.x) <= hb.x - margin &&
                    std::abs(local.z) <= hb.z - margin)
                {
                    glm::vec3 contactPoint =
                        baseCentroid - boxUp * (glm::dot(boxUp, baseCentroid) - boxTop);

                    glm::vec3 n = boxUp;
                    if (glm::dot(n, box.pos - pyr.pos) < 0.0f)
                        n = -n;

                    c.a = &pyr;
                    c.b = &box;
                    c.normal = n;
                    c.penetration = penetration;
                    c.point = contactPoint - n * GAP;

                    return true;
                }
            }
        }
    }

    
    // minpen
    

    float minPen = FLT_MAX;
    glm::vec3 bestAxis(0.0f, 1.0f, 0.0f);

    auto project = [&](const std::vector<glm::vec3>& v,
        const glm::vec3& axis,
        float& mn, float& mx)
        {
            mn = FLT_MAX; mx = -FLT_MAX;
            for (auto& p : v) {
                float d = glm::dot(p, axis);
                mn = std::min(mn, d);
                mx = std::max(mx, d);
            }
        };

    auto testAxis = [&](glm::vec3 axis) -> bool
        {
            if (glm::length2(axis) < 1e-8f) return true;
            axis = glm::normalize(axis);

            float a0, a1, b0, b1;
            project(boxVerts, axis, a0, a1);
            project(pyrVerts, axis, b0, b1);

            float overlap = std::min(a1, b1) - std::max(a0, b0);
            overlap -= GAP;

            if (overlap <= 0.0f) return false;

            if (overlap < minPen) {
                minPen = overlap;
                bestAxis = axis;
            }
            return true;
        };

    if (!testAxis(Rb[0])) return false;
    if (!testAxis(Rb[1])) return false;
    if (!testAxis(Rb[2])) return false;

    for (auto& f : pyrFaces)
        if (!testAxis(f.n)) return false;

    glm::vec3 n = glm::normalize(bestAxis);
    if (glm::dot(n, box.pos - pyr.pos) < 0.0f)
        n = -n;

    glm::vec3 contactPoint = 0.5f * (box.pos + pyr.pos) - n * GAP;

    c.a = &pyr;
    c.b = &box;
    c.normal = n;
    c.penetration = minPen;
    c.point = contactPoint;

    return true;
}





static bool boxBox(Shape& a, Shape& b, Contact& c) {
    glm::vec3 ha = getHalfExtents(a);
    glm::vec3 hb = getHalfExtents(b);

    glm::vec3 da = b.pos - a.pos;
    glm::vec3 overlap = ha + hb - glm::abs(da);

    if (overlap.x <= 0 || overlap.y <= 0 || overlap.z <= 0)
        return false;

    // eixo de menor pen
    if (overlap.x < overlap.y && overlap.x < overlap.z)
        c.normal = glm::vec3(glm::sign(da.x), 0, 0);
    else if (overlap.y < overlap.z)
        c.normal = glm::vec3(0, glm::sign(da.y), 0);
    else
        c.normal = glm::vec3(0, 0, glm::sign(da.z));

    c.penetration = std::min({ overlap.x, overlap.y, overlap.z });
    c.point = 0.5f * (a.pos + b.pos);
    c.a = &a;
    c.b = &b;

    return true;
}
static bool sphereBox(Shape& s, Shape& b, Contact& c) {
    glm::vec3 h = getHalfExtents(b);

    glm::vec3 rel = s.pos - b.pos;
    glm::vec3 closest = glm::clamp(rel, -h, h);
    glm::vec3 p = b.pos + closest;

    glm::vec3 d = s.pos - p;
    float dist2 = glm::length2(d);
    float r = sphereRadius(s);

    if (dist2 > r * r) return false;

    float dist = std::sqrt(dist2);
    glm::vec3 n = (dist > 1e-6f) ? d / dist : glm::vec3(0, 1, 0);

    c = { &b, &s, p, n, r - dist };
    return true;
}


void updatePhysics(Scene& scene, float dt) {
    if (dt <= 0.0f) return;
//integaracao
    for (auto& s : scene.spheresRef())  integrateVelocity(s, dt);
    for (auto& r : scene.rectsRef())    integrateVelocity(r, dt);
    for (auto& p : scene.pyramidsRef()) integrateVelocity(p, dt);

    // chao
    for (auto& s : scene.spheresRef())  handleGroundContact(s, dt);
    for (auto& r : scene.rectsRef())    handleGroundContact(r, dt);
    for (auto& p : scene.pyramidsRef()) handleGroundContact(p, dt);

    // se colidiu
    std::vector<Contact> contacts;

    auto& spheres = scene.spheresRef();
    auto& boxes = scene.rectsRef();
    auto& pyramids = scene.pyramidsRef();

    //bola bola
    for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = i + 1; j < spheres.size(); ++j) {
            Contact c;
            if (sphereSphere(spheres[i], spheres[j], c))
                contacts.push_back(c);
        }

    // bola box
    for (auto& s : spheres)
        for (auto& b : boxes) {
            Contact c;
            if (sphereBox(s, b, c))
                contacts.push_back(c);
        }

    // esfera triangulo
    for (auto& s : spheres)
        for (auto& p : pyramids) {
            Contact c;
            if (spherePyramid(s, p, c))
                contacts.push_back(c);
        }

    // box box
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            Contact c;
            if (boxBox(boxes[i], boxes[j], c))
                contacts.push_back(c);
        }

    // boxtringulo
    for (auto& b : boxes)
        for (auto& p : pyramids) {
            Contact c;
            if (boxPyramid(b, p, c))
                contacts.push_back(c);
        }

    
    for (int it = 0; it < 6; ++it)
        for (auto& c : contacts)
            solveContact(c, dt);
 
    for (auto& s : spheres) {
        integratePosition(s, dt);
        integrateRotation(s, dt);
    }
    for (auto& r : boxes) {
        integratePosition(r, dt);
        integrateRotation(r, dt);
    }
    for (auto& p : pyramids) {
        integratePosition(p, dt);
        integrateRotation(p, dt);
    }

    //sleep
    auto sleepCheck = [&](Shape& s) {
        if (s.isStatic || s.isDragging) return;

        if (glm::length(s.vel) < sleepLinearThreshold &&
            glm::length(s.angularVel) < sleepAngularThreshold) {

            s.sleepTimer += dt;
            if (s.sleepTimer > sleepTimeThreshold) {
                s.isSleeping = true;
                s.vel = {};
                s.angularVel = {};
            }
        }
        else {
            s.sleepTimer = 0.0f;
            s.isSleeping = false;
        }
        };

    for (auto& s : spheres)  sleepCheck(s);
    for (auto& r : boxes)    sleepCheck(r);
    for (auto& p : pyramids) sleepCheck(p);
}
