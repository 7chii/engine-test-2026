#include "physics.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cfloat>
#include <array>
#include <algorithm>
#include <limits>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>

//----------------------------------------------------------
// CONSTANTES
//----------------------------------------------------------

static const glm::vec3 GRAVITY(0.0f, -9.81f, 0.0f);
static const float restitutionGround = 0.2f;
static const float frictionGround = 0.8f;
static const float sleepTimeThreshold = 0.6f;


using BoxVerts = std::array<glm::vec3, 8>;
using PyramidVerts = std::array<glm::vec3, 5>;
using PyramidFaces = std::array<PyramidFace, 5>;

//----------------------------------------------------------
// BROADPHASE - AABB
//----------------------------------------------------------

struct AABB
{
    glm::vec3 min;
    glm::vec3 max;
};

//----------------------------------------------------------
// INÉRCIA
//----------------------------------------------------------

template<size_t N>
static void projectPoints(const std::array<glm::vec3, N>& pts,
    const glm::vec3& axis,
    float& outMin,
    float& outMax)
{
    outMin = FLT_MAX;
    outMax = -FLT_MAX;

    for (const auto& p : pts)
    {
        float proj = glm::dot(p, axis);
        outMin = std::min(outMin, proj);
        outMax = std::max(outMax, proj);
    }
}
// Retorna vértices em world space de uma box (ShapeType::Rect)
static void getBoxVertices(const Shape& box, BoxVerts& verts)
{
    glm::mat3 R = glm::mat3_cast(box.rot);
    glm::vec3 he = box.scale * 0.5f;

    int i = 0;
    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2)
                verts[i++] = box.pos + R * glm::vec3(x * he.x, y * he.y, z * he.z);
}
// Retorna vértices em world space da pirâmide

static void getPyramidVertices(const Shape& p, PyramidVerts& verts)
{
    glm::mat3 R = glm::mat3_cast(p.rot);

    float hx = p.scale.x * 0.5f;
    float hy = p.scale.y * 0.5f;
    float hz = p.scale.z * 0.5f;

    // Base em -hy
    verts[0] = p.pos + R * glm::vec3(-hx, -hy, -hz);
    verts[1] = p.pos + R * glm::vec3(hx, -hy, -hz);
    verts[2] = p.pos + R * glm::vec3(hx, -hy, hz);
    verts[3] = p.pos + R * glm::vec3(-hx, -hy, hz);

    // Apex em +hy
    verts[4] = p.pos + R * glm::vec3(0, hy, 0);
}

static AABB computeAABB(const Shape& s)
{
    if (s.type == ShapeType::Sphere)
    {
        float r = s.scale.x * s.radius; // substitui sphereRadius
        return { s.pos - glm::vec3(r), s.pos + glm::vec3(r) };
    }

    glm::vec3 minV(FLT_MAX);
    glm::vec3 maxV(-FLT_MAX);

    if (s.type == ShapeType::Rect)
    {
        BoxVerts verts;
        getBoxVertices(s, verts);

        for (const auto& v : verts)
        {
            minV = glm::min(minV, v);
            maxV = glm::max(maxV, v);
        }
    }
    else if (s.type == ShapeType::Pyramid)
    {
        std::array<glm::vec3, 5> verts;
        getPyramidVertices(s, verts);

        for (const auto& v : verts)
        {
            minV = glm::min(minV, v);
            maxV = glm::max(maxV, v);
        }
    }

    return { minV, maxV };
}

static bool aabbOverlap(const AABB& a, const AABB& b)
{
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}

static void getPyramidFaces(const Shape& p, PyramidFaces& faces)
{
    glm::mat3 R = glm::mat3_cast(p.rot);
    glm::vec3 he = p.scale * 0.5f;

    // Base (4 vértices)
    std::array<glm::vec3, 4> base = {
        p.pos + R * glm::vec3(-he.x, -he.y, -he.z),
        p.pos + R * glm::vec3(he.x, -he.y, -he.z),
        p.pos + R * glm::vec3(he.x, -he.y,  he.z),
        p.pos + R * glm::vec3(-he.x, -he.y,  he.z)
    };

    glm::vec3 apex = p.pos + R * glm::vec3(0, he.y, 0);

    // 4 faces laterais
    for (int i = 0; i < 4; ++i)
    {
        glm::vec3 a = base[i];
        glm::vec3 b = base[(i + 1) % 4];
        glm::vec3 c = apex;

        glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));

        // Garante que a normal aponta para fora
        if (glm::dot(n, p.pos - a) > 0.0f)
            n = -n;

        float d = -glm::dot(n, a);

        faces[i] = { n, d };
    }

    // Face da base (índice 4)
    glm::vec3 nBase = glm::normalize(R * glm::vec3(0, -1, 0));
    float dBase = -glm::dot(nBase, base[0]);

    faces[4] = { nBase, dBase };
}


// Calcula a matriz de inércia local baseada no tipo e escala do objeto.
glm::mat3 computeLocalInertiaTensor(const Shape& s)
{
    float m = s.mass;
    glm::vec3 size = s.scale;

    if (m <= 0.0f)
        return glm::mat3(0.0f);

    if (s.type == ShapeType::Rect)
    {
        float x2 = size.x * size.x;
        float y2 = size.y * size.y;
        float z2 = size.z * size.z;

        return glm::mat3(
            (1.0f / 12.0f) * m * (y2 + z2), 0, 0,
            0, (1.0f / 12.0f) * m * (x2 + z2), 0,
            0, 0, (1.0f / 12.0f) * m * (x2 + y2)
        );
    }

    if (s.type == ShapeType::Sphere)
    {
        float r = s.radius * std::max({ s.scale.x, s.scale.y, s.scale.z });
        float i = (2.0f / 5.0f) * m * r * r;
        return glm::mat3(i);
    }

    // Para pirâmide (base quadruada), aproxime como caixa (ajuste se necessrio)
    if (s.type == ShapeType::Pyramid)
    {
        float x2 = size.x * size.x;
        float y2 = size.y * size.y;
        float z2 = size.z * size.z;
        float Ixx = (3.0f / 20.0f) * m * (y2 + z2); // aproximação
        float Iyy = (3.0f / 20.0f) * m * (x2 + z2);
        float Izz = (3.0f / 20.0f) * m * (x2 + y2);
        return glm::mat3(
            Ixx, 0, 0,
            0, Iyy, 0,
            0, 0, Izz
        );
    }

    return glm::mat3(1.0f);
}

// Calcula matriz de inércia inversa no sistema de mundo
static glm::mat3 computeWorldInvInertia(const Shape& s)
{
    if (s.isStatic || s.isDragging)
        return glm::mat3(0.0f);

    glm::mat3 R = glm::mat3_cast(s.rot);
    return R * s.invInertia * glm::transpose(R);
}

//----------------------------------------------------------
// SLEEP THRESHOLDS dinâmicos
//----------------------------------------------------------
static float computeSleepLinearThreshold(const Shape& s) {
    // Linear: proporcional ao maior eixo
    float maxScale = std::max({ s.scale.x, s.scale.y, s.scale.z });
    return std::max(0.01f, std::min(0.2f, 0.05f * maxScale));
}
static float computeSleepAngularThreshold(const Shape& s) {
    // Angular: menos sensível ao tamanho, pode ser constante ou levemente dependente do scale
    float maxScale = std::max({ s.scale.x, s.scale.y, s.scale.z });
    return std::max(0.001f, 0.01f * maxScale);
}

//----------------------------------------------------------
// Funções utilitrias
//----------------------------------------------------------

inline void wake(Shape& s) {
    if (s.isStatic) return;
    s.isSleeping = false;
    s.sleepTimer = 0.0f;
}

static float sphereRadius(const Shape& s) {
    return s.radius * std::max({ s.scale.x, s.scale.y, s.scale.z });
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


static void getPyramidContactPoints(const Shape& s, std::vector<glm::vec3>& out) {
    //std::vector<glm::vec3> verts;
    PyramidVerts verts;
    getPyramidVertices(s, verts);

    out.clear();
    for (auto& v : verts)
        if (v.y <= 0.002f)
            out.push_back(v);
}

//----------------------------------------------------------
// INTEGRAÇÃO
//----------------------------------------------------------
static void integrateVelocity(Shape& s, float dt) {
    if (s.isStatic || s.isDragging || s.isSleeping) return;

    if (s.useGravity)
        s.vel += GRAVITY * dt;

    s.vel *= std::exp(-0.05f * dt);
    s.angularVel *= (1.0f - 0.15f * dt);
}

static void integratePosition(Shape& s, float dt) {
    if (s.isStatic || s.isDragging || s.isSleeping) return;
    s.pos += s.vel * dt;
}

static void integrateRotation(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging || s.isSleeping) return;

    glm::mat3 invInertiaWorld = computeWorldInvInertia(s);

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

//----------------------------------------------------------
// APLICAÇÃO DE IMPULSO
//----------------------------------------------------------
static void applyImpulse(Shape& s, const glm::vec3& J, const glm::vec3& cp) {
    if (s.isSleeping) {
        if (glm::length2(J) < 1e-6f)
            return; // ignora impulso minúsculo
        wake(s);
    }

    s.vel += J * getInvMass(s);

    glm::vec3 r = cp - s.pos;
    glm::vec3 torque = glm::cross(r, J);

    glm::mat3 invInertiaWorld = computeWorldInvInertia(s);
    s.angularVel += invInertiaWorld * torque;
}

//----------------------------------------------------------
// CONTATO COM O CHÃO
//----------------------------------------------------------
static void solveGroundContact(Shape& s, const glm::vec3& cp, float dt)
{
    if (s.isStatic) return;

    glm::vec3 n(0, 1, 0);
    float penetration = -cp.y;

    // -------------------------------------------------
    // 1. Sem penetração → nada a fazer
    // -------------------------------------------------
    if (penetration <= 0.0f)
        return;

    // -------------------------------------------------
    // 2. NÃO acordar por ruído microscópico
    // -------------------------------------------------
    const float WAKE_SPEED2 = 0.0025f; // ~5cm/s
    if (glm::length2(s.vel) > WAKE_SPEED2 ||
        glm::length2(s.angularVel) > WAKE_SPEED2)
    {
        wake(s);
    }

    // -------------------------------------------------
    // 3. Correção posicional (Baumgarte estvel)
    // -------------------------------------------------
    const float slop = 0.001f;
    const float percent = 0.6f;
    const float maxCorrection = 0.1f;

    float correction =
        std::min(maxCorrection,
            std::max(penetration - slop, 0.0f) * percent);

    s.pos.y += correction;

    // -------------------------------------------------
    // 4. Velocidade no ponto de contato
    // -------------------------------------------------
    glm::vec3 r = cp - s.pos;
    glm::vec3 vcp = s.vel + glm::cross(s.angularVel, r);
    float vn = glm::dot(vcp, n);

    // Se j est se afastando ou quase parado → não resolver
    if (vn >= -1e-4f)
        return;

    float invM = getInvMass(s);
    glm::mat3 invInertiaWorld = computeWorldInvInertia(s);

    float denom =
        invM +
        glm::dot(invInertiaWorld * glm::cross(r, n),
            glm::cross(r, n));

    if (denom < 1e-8f)
        return;

    // -------------------------------------------------
    // 5. Impulso normal
    // -------------------------------------------------
    float jn = -(1.0f + restitutionGround) * vn / denom;

    // Ignora impulso microscópico
    if (std::abs(jn) > 1e-6f)
        applyImpulse(s, jn * n, cp);

    // -------------------------------------------------
    // 6. Fricção Coulomb correta
    // -------------------------------------------------
    vcp = s.vel + glm::cross(s.angularVel, r);
    glm::vec3 vt = vcp - glm::dot(vcp, n) * n;
    float vtLen = glm::length(vt);

    if (vtLen > 1e-6f)
    {
        glm::vec3 t = -vt / vtLen;

        float denomT =
            invM +
            glm::dot(invInertiaWorld * glm::cross(r, t),
                glm::cross(r, t));

        if (denomT > 1e-8f)
        {
            float jt = -glm::dot(vcp, t) / denomT;

            float maxFriction = frictionGround * std::abs(jn);
            jt = glm::clamp(jt, -maxFriction, maxFriction);

            if (std::abs(jt) > 1e-6f)
                applyImpulse(s, jt * t, cp);
        }
    }

    // -------------------------------------------------
    // 7. Damping leve (sem destruir energia real)
    // -------------------------------------------------
    const float linearDamping = 0.98f;
    const float angularDamping = 0.98f;

    s.vel *= linearDamping;
    s.angularVel *= angularDamping;
}
static bool isCOMInsideBoxTopFace(const Shape& box, const Shape& obj)
{
    // eixo local da box
    glm::mat3 R = glm::mat3_cast(box.rot);

    glm::vec3 local = glm::transpose(R) * (obj.pos - box.pos);

    glm::vec3 half = box.scale * 0.5f;

    // testa projeção em X e Z da face superior
    return std::abs(local.x) <= half.x &&
        std::abs(local.z) <= half.z;
}

inline float quatAngularDistance(const glm::quat& q1, const glm::quat& q2) {
    float dot = glm::dot(glm::normalize(q1), glm::normalize(q2));
    dot = std::clamp(std::abs(dot), 0.0f, 1.0f); // protections
    return 2.0f * std::acos(dot);
}


// escolhe a face da pirâmide que melhor corresponde à normal de contato (maior dot)
// retorna true se encontrou e preenche centroid e faceNormal (ambos em world)
static bool findPyramidContactFaceCentroid(const Shape& pyr, const glm::vec3& contactNormal, glm::vec3& outCentroid, glm::vec3& outFaceNormal)
{
    //std::vector<glm::vec3> verts;
    PyramidVerts verts;
    getPyramidVertices(pyr, verts);
    if (verts.size() < 5) return false;

    // triângulos que compõem faces (base como 2 triângulos + 4 lados)
    struct Tri { int a, b, c; };
    Tri faces[6] = {
        {0,1,2}, {0,2,3}, // base (dois triângulos)
        {0,1,4},
        {1,2,4},
        {2,3,4},
        {3,0,4}
    };

    float bestDot = -FLT_MAX;
    int bestIdx = -1;
    glm::vec3 bestN(0.0f);

    // contactNormal aponta de suporte -> objeto (por convenção do seu solver).
    // a face correta da pirâmide ter a normal apontando aproximadamente na direção OPOSTA de contactNormal
    glm::vec3 wanted = -glm::normalize(contactNormal);

    for (int i = 0; i < 6; ++i)
    {
        glm::vec3 v0 = verts[faces[i].a];
        glm::vec3 v1 = verts[faces[i].b];
        glm::vec3 v2 = verts[faces[i].c];

        glm::vec3 n = glm::normalize(glm::cross(v1 - v0, v2 - v0));
        // orienta a normal para fora (consistência com getPyramidFaces)
        if (glm::dot(n, pyr.pos - v0) > 0.0f) n = -n;

        float d = glm::dot(n, wanted); // quanto a normal da face aponta para 'wanted' (-contactNormal)
        if (d > bestDot) {
            bestDot = d;
            bestIdx = i;
            bestN = n;
        }
    }

    if (bestIdx == -1) return false;

    // centróide do triângulo selecionado
    glm::vec3 v0 = verts[faces[bestIdx].a];
    glm::vec3 v1 = verts[faces[bestIdx].b];
    glm::vec3 v2 = verts[faces[bestIdx].c];
    outCentroid = (v0 + v1 + v2) / 3.0f;
    outFaceNormal = bestN;
    return true;
}

static void handleGroundContact(Shape& s, float dt) {
    bool wasGrounded = s.isGrounded;

    const float groundEps = 0.01f;

    const float stableTimeRequired = 0.5f;
    const float posEps = 0.01f;      // 1 centímetro para posição
    const float rotEps = 0.0175f;    // 1 grau para rotação

    
    const float GROUND_KEEP = 0.03f;   // 3 cm tolerância para manter
    const float GROUND_GAIN = 0.01f;
    float minY = FLT_MAX;
    glm::vec3 support;

    BoxVerts boxVerts;
    PyramidVerts pyrVerts;
    if (s.type == ShapeType::Sphere)
    {
        float r = s.scale.x * s.radius;
        minY = s.pos.y - r;
        support = glm::vec3(s.pos.x, minY, s.pos.z);
    }
    else if (s.type == ShapeType::Rect)
    {
        BoxVerts verts;
        getBoxVertices(s, verts);

        for (const auto& v : verts)
        {
            if (v.y < minY)
            {
                minY = v.y;
                support = v;
            }
        }
    }
    else if (s.type == ShapeType::Pyramid)
    {
        PyramidVerts verts;
        getPyramidVertices(s, verts);

        for (const auto& v : verts)
        {
            if (v.y < minY)
            {
                minY = v.y;
                support = v;
            }
        }
    }

    float threshold = wasGrounded ? GROUND_KEEP : GROUND_GAIN;
    bool touching = (minY <= threshold);
    /*
    if (s.type == ShapeType::Pyramid)
    {
        glm::vec3 faceCentroid, faceNormal;

        if (findPyramidContactFaceCentroid(s, glm::vec3(0, 1, 0), faceCentroid, faceNormal))
        {
            float dist = faceCentroid.y;
            float threshold = wasGrounded ? 0.03f : 0.01f;
            touching = (dist <= threshold);
        }
    }
    */

    float posDiff = glm::length(s.pos - s.lastGroundedPos);
    float rotDiff = quatAngularDistance(s.lastGroundedRot, s.rot);

    // Hysteresis: grounded se ficou estvel pelos thresholds, e só sai para bem além deles
    if (posDiff < posEps && rotDiff < rotEps) {
        s.groundedStableTimer += dt;
    }
    else if (posDiff > posEps * 3.0f || rotDiff > rotEps * 3.0f) {
        s.groundedStableTimer = 0.0f;
        s.lastGroundedPos = s.pos;
        s.lastGroundedRot = s.rot;
    }
    // se no meio, mantém o valor, não zera nem incrementa

    bool nearlyStable = s.groundedStableTimer > stableTimeRequired;

    //bool slowNearGround = (minY < (groundEps)) && std::abs(s.vel.y) < 0.1f;

    if (touching)
    {
        s.isGrounded = true;
    }
    else
    {
        // só perde grounded se realmente saiu do chão
        if (minY > GROUND_KEEP) {
            //printf(" ----retirado grounded ----- ");
            s.isGrounded = false;
    }
    }

    if (s.isGrounded)
    {
        glm::vec3 groundNormal(0, 1, 0);
        glm::vec3 faceNormal = glm::normalize(glm::mat3_cast(s.rot)[1]);

        float dotVal = glm::dot(faceNormal, groundNormal);

        const float ALIGN_EPS = 0.995f; // ~5 graus

        if (dotVal > ALIGN_EPS && !wasGrounded)
        {
            // força alinhamento perfeito
            glm::vec3 right = glm::normalize(glm::mat3_cast(s.rot)[0]);
            glm::vec3 forward = glm::cross(groundNormal, right);
            right = glm::cross(forward, groundNormal);

            glm::mat3 correctedBasis;
            correctedBasis[0] = glm::normalize(right);
            correctedBasis[1] = groundNormal;
            correctedBasis[2] = glm::normalize(forward);

            s.rot = glm::quat_cast(correctedBasis);

            s.angularVel = glm::vec3(0.0f);
        }
    }

    if (touching)
        solveGroundContact(s, support, dt);
    /*
    if (s.isGrounded && std::abs(s.vel.y) < 0.02f) {
        s.vel.y = 0.0f;
        if (glm::length2(s.vel) < 1e-6f)
            s.vel = glm::vec3(0.0f);
        s.angularVel = glm::vec3(0.0f);
    }
    */
    if (!wasGrounded && s.isGrounded)
        s.sleepTimer = 0.0f;
}

static bool isPointInsideBoxTopFace(const Shape& box, const glm::vec3& worldPoint)
{
    glm::mat3 R = glm::mat3_cast(box.rot);
    glm::vec3 local = glm::transpose(R) * (worldPoint - box.pos); // point em espaço local da box
    glm::vec3 half = box.scale * 0.5f;

    // checa projeção X/Z dentro dos half-extents da face superior
    // não exigimos que y esteja exatamente no topo; apenas que projete sobre a rea da face 1e-6f
    return (std::abs(local.x) <= half.x + 1e-6f && std::abs(local.z) <= half.z);
}

static bool isEffectivelyStable(const Shape& s)
{
    if (s.isStatic) return true;
    if (s.isSleeping) return true;
    if (s.isGrounded) return true;
    // pequenas folgas: se j começou a estabilizar (groundedStableTimer) ou est quase dormindo
    if (s.groundedStableTimer > 0.1f) return true;
    if (s.sleepTimer > 0.3f) return true;
    return false;
}

inline bool guessNormalAndPoint(const glm::vec3& a, const glm::vec3& b,
    glm::vec3& outSupportNormal, glm::vec3& outContactPoint, const Shape& s)
{
    // heurística: um dos argumentos provavelmente é uma normal unitria (~1.0 length)
    // e o outro é um ponto (distância razovel ao centro do shape).
    float la = glm::length(a);
    float lb = glm::length(b);

    // preferencias: se um tem comprimento ~1, trat-lo como normal
    if (std::abs(la - 1.0f) < 0.25f && lb > 0.001f) {
        outSupportNormal = glm::normalize(a);
        outContactPoint = b;
        return true;
    }
    if (std::abs(lb - 1.0f) < 0.25f && la > 0.001f) {
        outSupportNormal = glm::normalize(b);
        outContactPoint = a;
        return true;
    }

    // fallback: se um dos vetores est perto do centro do shape, assume ponto
    if (glm::length(a - s.pos) < glm::length(b - s.pos)) {
        outContactPoint = a;
        outSupportNormal = glm::normalize(b);
    }
    else {
        outContactPoint = b;
        outSupportNormal = glm::normalize(a);
    }
    return true;
}

inline void snapBoxToSurfaceA(Shape& s,
    const glm::vec3& argA,
    const glm::vec3& argB)
{
    if (s.isStatic) return;

    // detecta ordem (suporte normal vs contacto)
    glm::vec3 supportNormal, contactPoint;
    guessNormalAndPoint(argA, argB, supportNormal, contactPoint, s);

    // se normal invlida, aborta
    if (glm::length2(supportNormal) < 1e-8f) return;
    supportNormal = glm::normalize(supportNormal);

    // obtém normais das faces em world
    glm::mat3 R = glm::mat3_cast(s.rot);
    std::array<glm::vec3, 6> faceNormals = {
        R[0], -R[0],
        R[1], -R[1],
        R[2], -R[2]
    };

    // escolhe face mais alinhada com supportNormal
    int bestIdx = -1;
    float bestDot = -FLT_MAX;
    for (int i = 0; i < 6; ++i) {
        glm::vec3 fn = faceNormals[i];
        float d = glm::dot(fn, supportNormal);
        if (d > bestDot) {
            bestDot = d;
            bestIdx = i;
        }
    }

    // evita snap se face pouco alinhada
    const float MIN_ALIGN = 0.25f; // ajuste se quiser mais/menos permissivo
    if (bestIdx < 0 || bestDot < MIN_ALIGN) return;

    // se j praticamente alinhado, apenas corrige penetrações
    glm::vec3 chosenFrom = glm::normalize(faceNormals[bestIdx]);
    if (bestDot > 0.9995f) {
        // apenas empurra para evitar interpenetração com o plano definido por contactPoint+supportNormal
        //std::vector<glm::vec3> verts; 
        BoxVerts verts;
        getBoxVertices(s, verts);
        float minDist = FLT_MAX;
        for (auto& v : verts) {
            float d = glm::dot(v - contactPoint, supportNormal);
            minDist = std::min(minDist, d);
        }
        if (minDist < 0.0f) s.pos -= supportNormal * minDist;
        return;
    }

    // calcula rotação que leva a face escolhida -> supportNormal
    glm::quat q = glm::rotation(chosenFrom, supportNormal);
    q = glm::normalize(q);

    // rotaciona em torno do ponto de contato (mantém contato onde est)
    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);
    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);

    // corrige penetração pequena após rotação (garante que vértices não fiquem "dentro" do suporte)
    BoxVerts verts; 
    getBoxVertices(s, verts);
    float minDist = FLT_MAX;
    for (auto& v : verts) {
        float d = glm::dot(v - contactPoint, supportNormal);
        minDist = std::min(minDist, d);
    }
    if (minDist < 0.0f)
        s.pos -= supportNormal * minDist;

    // amortecimento para evitar jitter
    s.angularVel *= 0.25f;
    s.vel *= 0.5f;
}


inline void snapBoxToSurface(Shape& s,
    const glm::vec3& supportNormal,
    const glm::vec3& contactPoint)
{
    
    if (s.isStatic) return;

    glm::mat3 R = glm::mat3_cast(s.rot);

    std::array<glm::vec3, 6> faceNormals = {
        R[0], -R[0],
        R[1], -R[1],
        R[2], -R[2]
    };

    int bestIdx = -1;
    float bestDot = -FLT_MAX;

    for (int i = 0; i < 6; ++i)
    {
        float d = glm::dot(glm::normalize(faceNormals[i]), supportNormal);
        if (d > bestDot)
        {
            bestDot = d;
            bestIdx = i;
        }
    }

    const float MIN_ALIGN = 0.4f;
    if (bestDot < MIN_ALIGN)
        return;

    if (bestDot > 0.9995f)
        return;

    glm::vec3 from = glm::normalize(faceNormals[bestIdx]);
    glm::quat q = glm::rotation(from, supportNormal);
    q = glm::normalize(q);

    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);

    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);

    s.angularVel *= 0.2f;
    s.vel *= 0.4f;
}
inline void snapPyramidToSurface(Shape& s,
    const glm::vec3& supportNormal,
    const glm::vec3& contactPoint)
{
    if (s.isStatic) return;

    PyramidVerts verts;
    getPyramidVertices(s, verts);
    if (verts.size() < 5) return;

    struct Tri { int a, b, c; };
    Tri faces[6] = {
        {0,1,2}, {0,2,3},
        {0,1,4},
        {1,2,4},
        {2,3,4},
        {3,0,4}
    };

    int bestFace = -1;
    float bestDot = -FLT_MAX;
    glm::vec3 bestNormal(0);

    for (int i = 0; i < 6; ++i)
    {
        glm::vec3 v0 = verts[faces[i].a];
        glm::vec3 v1 = verts[faces[i].b];
        glm::vec3 v2 = verts[faces[i].c];

        glm::vec3 n = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        if (glm::dot(n, s.pos - v0) < 0.0f)
            n = -n;

        float d = glm::dot(n, supportNormal);

        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
            bestNormal = n;
        }
    }

    const float MIN_ALIGN = 0.4f;
    if (bestDot < MIN_ALIGN)
        return;

    glm::quat q = glm::rotation(bestNormal, supportNormal);
    q = glm::normalize(q);

    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);

    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);

    s.angularVel *= 0.2f;
    s.vel *= 0.4f;
}

inline void snapPyramidToSurfaceA(Shape& s,
    const glm::vec3& argA,
    const glm::vec3& argB)
{
    if (s.isStatic) return;

    //--------------------------------------------------
    // Detecta ordem (normal vs ponto)
    //--------------------------------------------------
    glm::vec3 supportNormal, contactPoint;
    guessNormalAndPoint(argA, argB, supportNormal, contactPoint, s);

    if (glm::length2(supportNormal) < 1e-8f)
        return;

    supportNormal = glm::normalize(supportNormal);

    //--------------------------------------------------
    // Obtém vértices da pirâmide
    //--------------------------------------------------
    PyramidVerts verts;
    getPyramidVertices(s, verts);
    if (verts.size() < 5) return;

    //--------------------------------------------------
    // Faces da pirâmide (base + 4 laterais)
    //--------------------------------------------------
    struct Tri { int a, b, c; };
    Tri faces[6] =
    {
        {0,1,2}, {0,2,3},   // base (quad dividido)
        {0,1,4},
        {1,2,4},
        {2,3,4},
        {3,0,4}
    };

    //--------------------------------------------------
    // Escolhe face mais alinhada ao suporte
    //--------------------------------------------------
    int bestFace = -1;
    float bestDot = -FLT_MAX;
    glm::vec3 bestNormal(0);

    for (int i = 0; i < 6; ++i)
    {
        glm::vec3 v0 = verts[faces[i].a];
        glm::vec3 v1 = verts[faces[i].b];
        glm::vec3 v2 = verts[faces[i].c];

        glm::vec3 n = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        // garante normal externa
        if (glm::dot(n, s.pos - v0) < 0.0f)
            n = -n;

        float d = glm::dot(n, supportNormal);

        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
            bestNormal = n;
        }
    }

    const float MIN_ALIGN = 0.25f;
    if (bestFace < 0 || bestDot < MIN_ALIGN)
        return;

    //--------------------------------------------------
    // Se j praticamente alinhado → só corrige penetração
    //--------------------------------------------------
    if (bestDot > 0.9995f)
    {
        float minDist = FLT_MAX;
        for (auto& v : verts)
        {
            float d = glm::dot(v - contactPoint, supportNormal);
            minDist = std::min(minDist, d);
        }

        if (minDist < 0.0f)
            s.pos -= supportNormal * minDist;

        return;
    }

    //--------------------------------------------------
    // Rotação da face escolhida → supportNormal
    //--------------------------------------------------
    glm::quat q = glm::rotation(bestNormal, supportNormal);
    q = glm::normalize(q);

    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);

    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);

    //--------------------------------------------------
    // Corrige penetração residual pós-rotação
    //--------------------------------------------------
    verts.empty();
    getPyramidVertices(s, verts);

    float minDist = FLT_MAX;
    for (auto& v : verts)
    {
        float d = glm::dot(v - contactPoint, supportNormal);
        minDist = std::min(minDist, d);
    }

    if (minDist < 0.0f)
        s.pos -= supportNormal * minDist;

    //--------------------------------------------------
    // Amortecimento para evitar jitter
    //--------------------------------------------------
    // trava movimento normal ao plano
    float vn = glm::dot(s.vel, supportNormal);
    if (vn < 0.0f)
        s.vel -= supportNormal * vn;

    // trava rotação residual pequena
    if (glm::length2(s.angularVel) < 0.0001f)
        s.angularVel = glm::vec3(0);
}
inline bool findSupportOnPlane(const Shape& s,
    const glm::vec3& planePoint,
    const glm::vec3& planeNormal,
    glm::vec3& outContactPoint,
    glm::vec3& outSupportNormal)
{
    const float SNAP_EPS = 0.02f;

    if (s.type == ShapeType::Sphere)
    {
        float radius = s.scale.x * s.radius;
        glm::vec3 bottom = s.pos - planeNormal * radius;

        float dist = glm::dot(bottom - planePoint, planeNormal);

        if (std::abs(dist) <= SNAP_EPS)
        {
            outContactPoint = bottom - planeNormal * dist;
            outSupportNormal = planeNormal;
            return true;
        }
        return false;
    }

    float minDist = FLT_MAX;

    if (s.type == ShapeType::Rect)
    {
        BoxVerts verts;
        getBoxVertices(s, verts);

        for (auto& v : verts)
        {
            float dist = glm::dot(v - planePoint, planeNormal);
            minDist = std::min(minDist, dist);
        }

        if (std::abs(minDist) > SNAP_EPS)
            return false;

        glm::vec3 sum(0.0f);
        int count = 0;

        for (auto& v : verts)
        {
            float dist = glm::dot(v - planePoint, planeNormal);
            if (std::abs(dist - minDist) <= 0.02f)
            {
                sum += v;
                count++;
            }
        }

        if (count == 0)
            return false;

        outContactPoint = sum / (float)count;
        outSupportNormal = planeNormal;
        return true;
    }

    if (s.type == ShapeType::Pyramid)
    {
        PyramidVerts verts;
        getPyramidVertices(s, verts);

        for (auto& v : verts)
        {
            float dist = glm::dot(v - planePoint, planeNormal);
            minDist = std::min(minDist, dist);
        }

        if (std::abs(minDist) > SNAP_EPS)
            return false;

        glm::vec3 sum(0.0f);
        int count = 0;

        for (auto& v : verts)
        {
            float dist = glm::dot(v - planePoint, planeNormal);
            if (std::abs(dist - minDist) <= 0.02f)
            {
                sum += v;
                count++;
            }
        }

        if (count == 0)
            return false;

        outContactPoint = sum / (float)count;
        outSupportNormal = planeNormal;
        return true;
    }

    return false;
}
inline bool findLocalSupport(const Shape& s,
    glm::vec3& outContactPoint,
    glm::vec3& outSupportNormal)
{
    const float GROUND_EPS = 0.01f;

    if (s.type == ShapeType::Sphere)
    {
        float radius = s.scale.x * s.radius;
        outContactPoint = s.pos - glm::vec3(0, radius, 0);
        outSupportNormal = glm::vec3(0, 1, 0);
        return outContactPoint.y <= GROUND_EPS;
    }

    float minY = FLT_MAX;

    if (s.type == ShapeType::Rect)
    {
        BoxVerts verts;
        getBoxVertices(s, verts);

        for (auto& v : verts)
            minY = std::min(minY, v.y);

        if (minY > GROUND_EPS)
            return false;

        glm::vec3 sum(0.0f);
        int count = 0;

        for (auto& v : verts)
        {
            if (v.y <= minY + 0.02f)
            {
                sum += v;
                count++;
            }
        }

        outContactPoint = sum / (float)count;
        outSupportNormal = glm::vec3(0, 1, 0);
        return true;
    }

    if (s.type == ShapeType::Pyramid)
    {
        PyramidVerts verts;
        getPyramidVertices(s, verts);

        for (auto& v : verts)
            minY = std::min(minY, v.y);

        if (minY > GROUND_EPS)
            return false;

        glm::vec3 sum(0.0f);
        int count = 0;

        for (auto& v : verts)
        {
            if (v.y <= minY + 0.02f)
            {
                sum += v;
                count++;
            }
        }

        outContactPoint = sum / (float)count;
        outSupportNormal = glm::vec3(0, 1, 0);
        return true;
    }

    return false;
}
//----------------------------------------------------------
// SOLUÇÃO DE CONTATO ENTRE CORPOS (CORRIGIDA)
//----------------------------------------------------------
// --- helpers para estabilidade entre box <-> pyramid ---------------------------------

static void solveContact(Contact& c, float dt) {
    Shape& A = *c.a; Shape& B = *c.b; 
    if (A.isStatic && B.isStatic) 
        return; 
    if (c.penetration > 0.001f)
    {
        wake(A);
        wake(B);
    }
    // --- Novidade: cheque de "support" estvel entre box <-> pyramid ---- 
    auto isStable = [](const Shape& s) { 
        bool stable = s.isStatic || s.isSleeping || s.isGrounded; 
        return stable; 
        }; 
    glm::vec3 up(0.0f, 1.0f, 0.0f); 
    glm::vec3 n = glm::normalize(c.normal); 
    if (glm::length2(n) < 1e-8f) 
        return; // normal aponta de A -> B (convenção seu Contact) 
    bool A_supports_B = isStable(A) && glm::dot(n, up) > 0.5f; 
    bool B_supports_A = isStable(B) && glm::dot(-n, up) > 0.5f; 
    // Se A é box e B é pirâmide: verificar centróide da face de B 
    if (A_supports_B && A.type == ShapeType::Rect && B.type == ShapeType::Rect) { 
        glm::vec3 faceCentroid, faceNormal; 
        if (findPyramidContactFaceCentroid(B, n, faceCentroid, faceNormal)) { // se centroide da face de pirâmide projeta sobre face superior da box -> grounded 
            if (isPointInsideBoxTopFace(A, faceCentroid)) { //printf("ponto rect rect "); //A.isGrounded = true; //A.isSleeping = true; 
                /*B.isGrounded = true;
                B.isSleeping = true; // atualiza histórico de grounded para hysteresis/stable detection 
                B.lastGroundedPos = B.pos; 
                B.lastGroundedRot = B.rot; */

                B.isGrounded = true;
                A.isGrounded = true;

                B.isSleeping = true;
                A.isSleeping = true;

                B.vel = glm::vec3(0.0f);
                B.angularVel = glm::vec3(0.0f);

                A.vel = glm::vec3(0.0f);
                A.angularVel = glm::vec3(0.0f);

                B.lastGroundedPos = B.pos;
                B.lastGroundedRot = B.rot;

                A.lastGroundedPos = A.pos;
                A.lastGroundedRot = A.rot;
            } 
            else { //printf("permite tombar/rotacionar rect"); // permite tombar/rotacionar (não marcar como grounded) 
                B.isGrounded = false; 
                A.isGrounded = true; //A.isSleeping = true; 
        } 
        } 
        else { //printf("sem face detectada"); // sem face detectada, fallback: comportamento padrão (não forçar grounded) 
            B.isGrounded = false; 
        } 
    } if (A_supports_B && A.type == ShapeType::Rect && B.type == ShapeType::Pyramid ) {
        glm::vec3 faceCentroid, faceNormal; 
        if (findPyramidContactFaceCentroid(B, n, faceCentroid, faceNormal)) { // se centróide da face de pirâmide projeta sobre face superior da box -> grounded 
            if (isPointInsideBoxTopFace(A, faceCentroid)) { //A.isGrounded = true; //A.isSleeping = true; 
                /*B.isGrounded = true;
                B.isSleeping = true; // atualiza histórico de grounded para hysteresis/stable detection 
                B.lastGroundedPos = B.pos; 
                B.lastGroundedRot = B.rot; 

                */

                B.isGrounded = true;
                A.isGrounded = true;

                B.isSleeping = true;
                A.isSleeping = true;

                B.vel = glm::vec3(0.0f);
                B.angularVel = glm::vec3(0.0f);

                A.vel = glm::vec3(0.0f);
                A.angularVel = glm::vec3(0.0f);

                B.lastGroundedPos = B.pos;
                B.lastGroundedRot = B.rot;

                A.lastGroundedPos = A.pos;
                A.lastGroundedRot = A.rot;

            } else { //printf("permite tombar/rotacionar rectpir"); // permite tombar/rotacionar (não marcar como grounded) 
                B.isGrounded = false; 
                A.isGrounded = true; //A.isSleeping = true; 
            } 
        } else { //printf("sem face detectada"); // sem face detectada, fallback: comportamento padrão (não forçar grounded) 
            B.isGrounded = false; 
        }
    } // Se B é box e A é pirâmide (o caso invertido): mesma lógica simétrica 
    else if (A_supports_B && A.type == ShapeType::Rect && B.type == ShapeType::Sphere)
    {
        const float VN_EPS = 0.05f;

        const float VT_EPS = 0.09f;
        const float VEL_EPS = 0.09f;
        const float ANG_EPS = 0.09f;

        // velocidades no ponto de contato
        glm::vec3 rA = c.point - A.pos;
        glm::vec3 rB = c.point - B.pos;

        glm::vec3 vA = A.vel + glm::cross(A.angularVel, rA);
        glm::vec3 vB = B.vel + glm::cross(B.angularVel, rB);

        glm::vec3 rv = vB - vA;

        float vn = glm::dot(rv, n);
        glm::vec3 vt = rv - vn * n;
        float vtLen = glm::length(vt);

        bool lowNormalMotion = std::abs(vn) < VN_EPS; //unico menor

        bool lowTangentialMotion = vtLen < VT_EPS;
        bool lowLinearVel =
            glm::length(B.vel) < VEL_EPS &&
            glm::length(A.vel) < VEL_EPS;
        bool lowAngularVel =
            glm::length(B.angularVel) < ANG_EPS &&
            glm::length(A.angularVel) < ANG_EPS;


        bool stable =
            lowNormalMotion &&
            lowTangentialMotion &&
            lowLinearVel &&
            lowAngularVel;

        if (stable)
        {
            B.isGrounded = true;
            A.isGrounded = true;

            B.isSleeping = true;
            A.isSleeping = true;

            B.vel = glm::vec3(0.0f);
            B.angularVel = glm::vec3(0.0f);

            A.vel = glm::vec3(0.0f);
            A.angularVel = glm::vec3(0.0f);

            B.lastGroundedPos = B.pos;
            B.lastGroundedRot = B.rot;

            A.lastGroundedPos = A.pos;
            A.lastGroundedRot = A.rot;
        }
        else
        {
            B.isGrounded = true;   // ainda apoiada
            B.isSleeping = false;
        }
    }
        else if (B_supports_A && B.type == ShapeType::Rect && A.type == ShapeType::Pyramid) {
        glm::vec3 faceCentroid, faceNormal;
        if (findPyramidContactFaceCentroid(A, -n, faceCentroid, faceNormal)) {
            if (isPointInsideBoxTopFace(B, faceCentroid)) {
                A.isGrounded = true;
                A.lastGroundedPos = A.pos;
                A.lastGroundedRot = A.rot;
            }
            else {
                A.isGrounded = false;
            }
        }
        else {
            A.isGrounded = false;
        }
    } else { // casos gerais: mantemos a heurística anterior (se o suporte for "estvel", marca grounded) 
            if (A_supports_B) B.isGrounded = true; 
            if (B_supports_A) A.isGrounded = true; 
    } float invMA = getInvMass(A); 
    float invMB = getInvMass(B); 
    glm::mat3 invIA = computeWorldInvInertia(A); 
    glm::mat3 invIB = computeWorldInvInertia(B); 
    glm::vec3 rA = c.point - A.pos; 
    glm::vec3 rB = c.point - B.pos; 
    glm::vec3 vA = A.vel + glm::cross(A.angularVel, rA); 
    glm::vec3 vB = B.vel + glm::cross(B.angularVel, rB); 
    glm::vec3 rv = vB - vA; float vn = glm::dot(rv, n); // Correção posicional (projeção) 
            { const float slop = 0.001f; 
            const float percent = 0.7f; 
            float pen = std::max(c.penetration - slop, 0.0f); 
            float denom = invMA + invMB; 
            if (denom > 0.0f) { 
                glm::vec3 corr = (pen * percent / denom) * n; 
                if (!A.isStatic) A.pos -= corr * invMA; 
                if (!B.isStatic) B.pos += corr * invMB; 
            } 
    } // se corpos se separando ao longo da normal, não aplica impulso normal 
            if (vn > 0.0f) return; 
            float e = restitutionGround; 
            glm::vec3 rAxn = glm::cross(rA, n);
            glm::vec3 rBxn = glm::cross(rB, n);
            float denomN = invMA + invMB + glm::dot(invIA * rAxn, rAxn) + glm::dot(invIB * rBxn, rBxn);
            if (denomN <= 0.0f) return; 
            float jn = -(1.0f + e) * vn / denomN; 
            glm::vec3 impulseN = jn * n; 
            if (!A.isStatic) { 
                A.vel -= impulseN * invMA; 
                A.angularVel -= invIA * glm::cross(rA, impulseN); 
            } 
            if (!B.isStatic) { 
                B.vel += impulseN * invMB; 
                B.angularVel += invIB * glm::cross(rB, impulseN); 
            } // --- Fricção tangencial (Coulomb) --- 
            rv = (B.vel + glm::cross(B.angularVel, rB)) - (A.vel + glm::cross(A.angularVel, rA)); 
            glm::vec3 t = rv - glm::dot(rv, n) * n; 
            float tLen = glm::length(t); 
            if (tLen < 1e-6f) return; 
            t /= tLen; 
            glm::vec3 rAt = glm::cross(rA, t);
            glm::vec3 rBt = glm::cross(rB, t);
            float denomT = invMA + invMB + glm::dot(invIA * rAt, rAt) + glm::dot(invIB * rBt, rBt);
            if (denomT <= 0.0f) return; 
            float jt = -glm::dot(rv, t) / denomT; 
            float maxFriction = frictionGround * jn; 
            jt = glm::clamp(jt, -maxFriction, maxFriction); 
            glm::vec3 impulseT = jt * t; 
            if (!A.isStatic) { 
                A.vel -= impulseT * invMA; 
                A.angularVel -= invIA * glm::cross(rA, impulseT); 
            } if (!B.isStatic) { 
                B.vel += impulseT * invMB; 
                B.angularVel += invIB * glm::cross(rB, impulseT);
            } 
}

// --- Sphere x Pyramid (face contact robusta) ---
static bool spherePyramid(Shape& sphere, Shape& pyr, Contact& c) {
    float r = sphereRadius(sphere);

    PyramidFaces faces;
    getPyramidFaces(pyr, faces);

    float maxSeparation = -FLT_MAX;
    int maxIdx = -1;
    glm::vec3 sphereCenter = sphere.pos;

    // Encontre a face mais "profunda" (maxima separacao)
    for (int i = 0; i < (int)faces.size(); ++i) {
        float d = glm::dot(faces[i].n, sphereCenter) + faces[i].d;
        if (d > r) {
            // Esfera esta do lado de fora dessa face -- NÃO h contato!
            return false;
        }
        if (d > maxSeparation) {
            maxSeparation = d;
            maxIdx = i;
        }
    }

    // O ponto de contato estar na direcao da normal da face mais distante
    const PyramidFace& refFace = faces[maxIdx];
    glm::vec3 contactNormal = refFace.n;
    float penetration = r - maxSeparation;
    glm::vec3 contactPoint = sphereCenter - contactNormal * r;

    c.a = &pyr;
    c.b = &sphere;
    c.point = contactPoint;
    c.normal = contactNormal;
    c.penetration = penetration;

    return true;
}

//----------------------------------------------------------
// DETECÇÃO DE COLISÕES
//----------------------------------------------------------
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
// ---- BOX-BOX SAT ----
static bool boxBox(const Shape& a, const Shape& b, Contact& c)
{
    BoxVerts vertsA;
    BoxVerts vertsB;

    getBoxVertices(a, vertsA);
    getBoxVertices(b, vertsB);

    glm::mat3 Ra = glm::mat3_cast(a.rot);
    glm::mat3 Rb = glm::mat3_cast(b.rot);

    float minOverlap = FLT_MAX;
    glm::vec3 bestAxis;

    auto testAxis = [&](const glm::vec3& axis0) -> bool
        {
            if (glm::length2(axis0) < 1e-6f)
                return true;

            glm::vec3 axis = axis0;

            float minA, maxA, minB, maxB;
            projectPoints(vertsA, axis, minA, maxA);
            projectPoints(vertsB, axis, minB, maxB);

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < 0)
                return false;

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis *
                    (glm::dot(b.pos - a.pos, axis) >= 0 ? 1.0f : -1.0f);
            }

            return true;
        };

    // 3 face axes A
    for (int i = 0; i < 3; ++i)
        if (!testAxis(Ra[i])) return false;

    // 3 face axes B
    for (int i = 0; i < 3; ++i)
        if (!testAxis(Rb[i])) return false;

    // 9 cross axes
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!testAxis(glm::cross(Ra[i], Rb[j])))
                return false;

    c.a = const_cast<Shape*>(&a);
    c.b = const_cast<Shape*>(&b);
    c.normal = glm::normalize(bestAxis);
    c.penetration = minOverlap;
    c.point = 0.5f * (a.pos + b.pos);

    return true;
}


// ---- BOX-PYRAMID SAT ----
static bool boxPyramid(const Shape& box,
    const Shape& pyr,
    Contact& c)
{
    BoxVerts vertsBox;
    PyramidVerts vertsPyr;

    getBoxVertices(box, vertsBox);
    getPyramidVertices(pyr, vertsPyr);

    glm::mat3 R = glm::mat3_cast(box.rot);

    PyramidFaces pyrFaces;
    getPyramidFaces(pyr, pyrFaces);

    float minOverlap = FLT_MAX;
    glm::vec3 bestAxis;

    auto testAxis = [&](const glm::vec3& axis0) -> bool
        {
            if (glm::length2(axis0) < 1e-6f)
                return true;

            glm::vec3 axis = axis0;

            float minA, maxA, minB, maxB;
            projectPoints(vertsBox, axis, minA, maxA);
            projectPoints(vertsPyr, axis, minB, maxB);

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < 0)
                return false;

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis *
                    (glm::dot(pyr.pos - box.pos, axis) >= 0 ? 1.0f : -1.0f);
            }

            return true;
        };

    // 3 box face axes
    for (int i = 0; i < 3; ++i)
        if (!testAxis(R[i])) return false;

    // 5 pyramid face axes
    for (int i = 0; i < 5; ++i)
        if (!testAxis(pyrFaces[i].n)) return false;

    // cross products
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 5; ++j)
            if (!testAxis(glm::cross(R[i], pyrFaces[j].n)))
                return false;

    c.a = const_cast<Shape*>(&box);
    c.b = const_cast<Shape*>(&pyr);
    c.normal = glm::normalize(bestAxis);
    c.penetration = minOverlap;
    c.point = 0.5f * (box.pos + pyr.pos);

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
inline void sleepCheck(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging) {
        //printf("** static + dragging** ");
        //printf("%s", s.isSleeping);
        return;
    }

    // =============================
    // CONFIGURAÇÃO
    // =============================

    const float LIN_SLEEP_EPS = 0.03f;
    const float ANG_SLEEP_EPS = 0.03f;     // tolerância angular
    const float HARD_RESET_SPEED = 0.05f; // 5 cm/s real
    const float HARD_RESET_SPEED2 = HARD_RESET_SPEED * HARD_RESET_SPEED;  // movimento real
    const float SLEEP_TIME = 0.6f;
    const float MIN_PARALLEL = 0.995f;

    // =============================
    // Se não est grounded, nunca dorme
    // =============================

    if (!s.isGrounded)
    {
        //printf(" **not grounded** ");
        s.sleepTimer = 0.0f;
        s.isSleeping = false;
        return;
    }

    float lin2 = glm::length2(s.vel);
    float ang2 = glm::length2(s.angularVel);

    // =============================
    // TESTE ANGULAR (quase paralelo ao chão)
    // =============================

    glm::vec3 groundNormal(0, 1, 0);
    glm::vec3 supportPoint, supportNormal;

    bool hasSupport = findLocalSupport(s, supportPoint, supportNormal);

    bool isNearlyFlat = false;

    if (hasSupport)
    {
        float dotVal = glm::dot(glm::normalize(supportNormal), groundNormal);

        // 0.95 ≈ até ~18 graus de inclinação
        const float ALIGN_THRESHOLD = 0.95f;

        isNearlyFlat = dotVal > ALIGN_THRESHOLD;
    }

    if (!s.isGrounded && !isNearlyFlat)
    {
        //printf(" **nearlyflat** ");
        s.sleepTimer = 0.0f;
        s.isSleeping = false;
        return;
    }

    // =============================
    // SNAP PREVENTIVO (opcional mas recomendado)
    // =============================

    if (ang2 < 4e-4f)
    {
        glm::vec3 contactPoint, supportNormal;

        if (findLocalSupport(s, contactPoint, supportNormal))
        {
            if (s.type == ShapeType::Rect)
                snapBoxToSurface(s, supportNormal, contactPoint);
            else if (s.type == ShapeType::Pyramid)
                snapPyramidToSurface(s, supportNormal, contactPoint);
        }
    }

    // =============================
    // TESTE DE MOVIMENTO
    // =============================

    bool slowEnough =
        lin2 < LIN_SLEEP_EPS * LIN_SLEEP_EPS &&
        ang2 < ANG_SLEEP_EPS * ANG_SLEEP_EPS;
    //printf(" lin2 : %lf, ang2 : %lf ", lin2, ang2);
    //printf(" %d ", slowEnough);
    if (slowEnough)
    {
        // força zero absoluto para matar jitter
        s.vel = glm::vec3(0.0f);
        s.angularVel = glm::vec3(0.0f);

        s.sleepTimer += dt;

        if (s.sleepTimer >= SLEEP_TIME)
        {
            s.isSleeping = true;
            return;
        }
    }
    else
    {
        // Só resetar se movimento for REAL
        if (!s.isGrounded &&
            (lin2 > HARD_RESET_SPEED2 || ang2 > HARD_RESET_SPEED2))
        {
            //printf(" ** hard reset ** ");
            s.sleepTimer = 0.0f;
            s.isSleeping = false;
        }
    }
}

//----------------------------------------------------------
// UPDATE PHYSICS PRINCIPAL
//----------------------------------------------------------
//----------------------------------------------------------
// UPDATE PHYSICS PRINCIPAL (COM BROADPHASE AABB)
//----------------------------------------------------------
void updatePhysics(Scene& scene, float dt)
{
    if (dt <= 0.0f) return;

    auto& spheres = scene.spheresRef();
    auto& boxes = scene.rectsRef();
    auto& pyramids = scene.pyramidsRef();

    // -------------------------------------------------
    // 1) Integrar velocidade
    // -------------------------------------------------
    for (auto& s : spheres)  integrateVelocity(s, dt);
    for (auto& b : boxes)    integrateVelocity(b, dt);
    for (auto& p : pyramids) integrateVelocity(p, dt);

    // -------------------------------------------------
    // 2) Contato com chão
    // -------------------------------------------------
    for (auto& s : spheres)  handleGroundContact(s, dt);
    for (auto& b : boxes)    handleGroundContact(b, dt);
    for (auto& p : pyramids) handleGroundContact(p, dt);

    // -------------------------------------------------
    // 3) Broadphase AABB
    // -------------------------------------------------

    std::vector<AABB> sphereAABB(spheres.size());
    std::vector<AABB> boxAABB(boxes.size());
    std::vector<AABB> pyramidAABB(pyramids.size());

    for (size_t i = 0; i < spheres.size(); ++i)
        sphereAABB[i] = computeAABB(spheres[i]);

    for (size_t i = 0; i < boxes.size(); ++i)
        boxAABB[i] = computeAABB(boxes[i]);

    for (size_t i = 0; i < pyramids.size(); ++i)
        pyramidAABB[i] = computeAABB(pyramids[i]);

    // -------------------------------------------------
    // 4) Narrowphase (com filtro AABB)
    // -------------------------------------------------

    Contact c;

    // -------- esfera-esfera --------
    for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = i + 1; j < spheres.size(); ++j)
        {
            if (!aabbOverlap(sphereAABB[i], sphereAABB[j]))
                continue;

            if (sphereSphere(spheres[i], spheres[j], c))
                solveContact(c, dt);
        }

    // -------- esfera-box --------
    for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = 0; j < boxes.size(); ++j)
        {
            if (!aabbOverlap(sphereAABB[i], boxAABB[j]))
                continue;

            if (sphereBox(spheres[i], boxes[j], c))
                solveContact(c, dt);
        }

    // -------- box-box --------
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = i + 1; j < boxes.size(); ++j)
        {
            if (!aabbOverlap(boxAABB[i], boxAABB[j]))
                continue;

            if (boxBox(boxes[i], boxes[j], c))
                solveContact(c, dt);
        }

    // -------- box-pirâmide --------
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = 0; j < pyramids.size(); ++j)
        {
            if (!aabbOverlap(boxAABB[i], pyramidAABB[j]))
                continue;

            if (boxPyramid(boxes[i], pyramids[j], c))
                solveContact(c, dt);
        }

    // -------- esfera-pirâmide --------
    for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = 0; j < pyramids.size(); ++j)
        {
            if (!aabbOverlap(sphereAABB[i], pyramidAABB[j]))
                continue;

            if (spherePyramid(spheres[i], pyramids[j], c))
                solveContact(c, dt);
        }

    // -------------------------------------------------
    // 5) Integrar posição e rotação
    // -------------------------------------------------
    for (auto& s : spheres) {
        integratePosition(s, dt);
        integrateRotation(s, dt);
    }

    for (auto& b : boxes) {
        integratePosition(b, dt);
        integrateRotation(b, dt);
    }

    for (auto& p : pyramids) {
        integratePosition(p, dt);
        integrateRotation(p, dt);
    }

    // -------------------------------------------------
    // 6) Sleep check
    // -------------------------------------------------
    for (auto& s : spheres)  sleepCheck(s, dt);
    for (auto& b : boxes)    sleepCheck(b, dt);
    for (auto& p : pyramids) sleepCheck(p, dt);
}