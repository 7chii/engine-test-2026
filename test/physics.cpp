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

static const float WAKE_SPEEDANGVEL = 0.000000001f;

static const float WAKE_SPEEDVEL = 0.000000000001f;



//adicionar motor de friccao que possa ser alterado conforme material  add material ao chao

/*
using BoxVerts = std::array<glm::vec3, 8>;
using PyramidVerts = std::array<glm::vec3, 5>;
using PyramidFaces = std::array<PyramidFace, 5>;
*/
//----------------------------------------------------------
// BROADPHASE - AABB
//----------------------------------------------------------

struct AABB
{
    glm::vec3 min;
    glm::vec3 max;
};


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
static bool isEffectivelyStable(const Shape& s)
{
    //printf("-- shape : %d , stabletimer : %lf --",s.id, s.groundedStableTimer);

    if (s.isStatic) return true;
    if (s.isSleeping) return true;
    
    //if (s.vel.value < 0.01f && !s.isSleeping) return true;
    //if (s.isGrounded) return true;
    // pequenas folgas: se j começou a estabilizar (groundedStableTimer) ou est quase dormindo
    if (s.groundedStableTimer > 0.5f) return true;
    if (s.sleepTimer > 0.3f) return true;
    return false;
}

// Retorna vertices em world space de uma box (ShapeType::Rect)
static void computeBoxVertices(Shape& s)
{
    glm::mat3 R = glm::mat3_cast(s.rot);
    glm::vec3 he = s.scale * 0.5f;
    s.geometry.vertexCount = 8; // box

    int i = 0;

    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2)
                s.geometry.vertices[i++] =
                s.pos + R * glm::vec3(x * he.x, y * he.y, z * he.z);
}
// Retorna vertices em world space da piramide

// Retorna vertices em world space da piramide
static void computePyramidVertices(Shape& p)
{
    // Segurança (estilo console determinístico)
    if (p.geometry.vertexCount < 5)
        return;

    glm::mat3 R = glm::mat3_cast(p.rot);
    p.geometry.vertexCount = 5;
    p.geometry.faceCount = 5;

    const float hx = p.scale.x * 0.5f;
    const float hy = p.scale.y * 0.5f;
    const float hz = p.scale.z * 0.5f;

    // Base (y = -hy)
    p.geometry.vertices[0] = p.pos + R * glm::vec3(-hx, -hy, -hz);
    p.geometry.vertices[1] = p.pos + R * glm::vec3(hx, -hy, -hz);
    p.geometry.vertices[2] = p.pos + R * glm::vec3(hx, -hy, hz);
    p.geometry.vertices[3] = p.pos + R * glm::vec3(-hx, -hy, hz);

    // Apex (y = +hy)
    p.geometry.vertices[4] = p.pos + R * glm::vec3(0.0f, hy, 0.0f);
}
static AABB computeAABB(const Shape& s)
{
    if (s.type == ShapeType::Sphere)
    {
        float r = s.scale.x * s.radius;
        return { s.pos - glm::vec3(r), s.pos + glm::vec3(r) };
    }

    glm::vec3 minV(FLT_MAX);
    glm::vec3 maxV(-FLT_MAX);

    // Usa vertices persistentes
    for (int i = 0; i < s.geometry.vertexCount; ++i)
    {
        const glm::vec3& v = s.geometry.vertices[i];

        minV = glm::min(minV, v);
        maxV = glm::max(maxV, v);
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
static void computePyramidFaces(Shape& p)
{
    if (p.geometry.vertexCount < 5)
        return;

    auto& faces = p.geometry.faces;
    p.geometry.faceCount = 5;

    const auto& v = p.geometry.vertices;

    // 4 laterais
    for (int i = 0; i < 4; ++i)
    {
        const glm::vec3& a = v[i];
        const glm::vec3& b = v[(i + 1) % 4];
        const glm::vec3& c = v[4];

        glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));

        if (glm::dot(n, p.pos - a) > 0.0f)
            n = -n;

        faces[i].n = n;
        faces[i].d = -glm::dot(n, a);
    }

    // base
    glm::vec3 nBase = glm::normalize(glm::cross(v[1] - v[0], v[3] - v[0]));
    faces[4].n = nBase;
    faces[4].d = -glm::dot(nBase, v[0]);
}

static void updateShapeWorldGeometry(Shape& s)
{
    switch (s.type)
    {
    case ShapeType::Rect:
        computeBoxVertices(s);
        break;

    case ShapeType::Pyramid:
        computePyramidVertices(s);
        computePyramidFaces(s);
        break;

    default:
        break;
    }
}

// Calcula a matriz de inercia local baseada no tipo e escala do objeto.
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

    // Para piramide (base quadruada), aproxime como caixa (ajuste se necessrio)
    if (s.type == ShapeType::Pyramid)
    {
        float x2 = size.x * size.x;
        float y2 = size.y * size.y;
        float z2 = size.z * size.z;
        float Ixx = (3.0f / 20.0f) * m * (y2 + z2); // aproximaao
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

// Calcula matriz de inercia inversa no sistema de mundo
static glm::mat3 computeWorldInvInertia(const Shape& s)
{
    if (s.isStatic || s.isDragging )
        return glm::mat3(0.0f);

    glm::mat3 R = glm::mat3_cast(s.rot);
    return R * s.invInertia * glm::transpose(R);
}

//----------------------------------------------------------
// SLEEP THRESHOLDS dinamicos
//----------------------------------------------------------
static float computeSleepLinearThreshold(const Shape& s) {
    // Linear: proporcional ao maior eixo
    float maxScale = std::max({ s.scale.x, s.scale.y, s.scale.z });
    return std::max(0.01f, std::min(0.2f, 0.05f * maxScale));
}
static float computeSleepAngularThreshold(const Shape& s) {
    // Angular: menos sensivel ao tamanho, pode ser constante ou levemente dependente do scale
    float maxScale = std::max({ s.scale.x, s.scale.y, s.scale.z });
    return std::max(0.001f, 0.01f * maxScale);
}

//----------------------------------------------------------
// Funcoes utilitrias
//----------------------------------------------------------

inline void wake(Shape& s) {
   // if (s.isStatic) return;
    s.isSleeping = false;
    s.sleepTimer = 0.0f;
    //s.groundedStableTimer = 0.0f;
}


static void wakeIfThresholdPassed(Shape& s) {
    wake(s);
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
    out.clear();
    for (int i = 0; i < s.geometry.vertexCount; ++i)
    {
        const glm::vec3& v = s.geometry.vertices[i];

        if (v.y <= 0.002f)
            out.push_back(v);
    }
}

//----------------------------------------------------------
// INTEGRACAO
//----------------------------------------------------------
static void integrateGravity(Shape& s, float dt)
{
    //s.isStatic  ||
    //wake(s);
    
        
        constexpr float ANGULAR_EPS2 = 1e-8f;
        //float angVel2 = glm::length2(s.angularVel);

        if (s.useGravity) {

            s.vel += GRAVITY * dt;
        }


        s.vel *= std::exp(-0.05f * dt);

        // -------------------------------------------------
        // DAMPING ANGULAR
        // -------------------------------------------------
        s.angularVel *= (1.0f - 0.15f * dt);
    

}

static void integratePosition(Shape& s, float dt)
{
    //s.isStatic || s.isDragging ||
    //if ( s.isDragging )
      //  return;

    constexpr float LIN_EPS2 = 0.0f;
    constexpr float ANG_EPS2 = 1e-8f;

    float vel2 = glm::length2(s.vel);
    float ang2 = glm::length2(s.angularVel);


    // -------------------------------------------------
    // Integra posição
    // -------------------------------------------------
    s.pos += s.vel * dt;


    s.geometryDirty = true;

}
static void integrateRotation(Shape& s, float dt)
{
    //s.isStatic || s.isDragging ||
    if (s.isDragging )
        return;

    constexpr float ANGULAR_EPS2 = 1e-8f;

    float w2 = glm::length2(s.angularVel);

    // Se quase parado → zera ruído
    if (w2 < ANGULAR_EPS2)
    {
        s.angularVel = glm::vec3(0.0f);
        return;
    }

    // -------------------------------------------------
    // Integra rotação
    // -------------------------------------------------
    float w = std::sqrt(w2);
    glm::vec3 axis = s.angularVel / w;

    glm::quat dq = glm::angleAxis(w * dt, axis);

    s.rot = glm::normalize(dq * s.rot);

    // marcou que mudou geometria
    s.geometryDirty = true;
}
/*
const float WAKE_SPEED2 = 0.0025f; // ~5cm/s
    if (glm::length2(s.vel) > WAKE_SPEED2 ||
        glm::length2(s.angularVel) > WAKE_SPEED2)
    {
        wake(s);
    }*/
//----------------------------------------------------------
// 
// 
// APLICACAO TODO IMPULSO
//----------------------------------------------------------
static void applyImpulse(Shape& s, const glm::vec3& J, const glm::vec3& cp) {
    wake(s);
    s.groundedStableTimer = 0.0f;
    //s.sleepTimer = 0.0f;
    s.vel += J * getInvMass(s);

    glm::vec3 r = cp - s.pos;
    glm::vec3 torque = glm::cross(r, J);

    glm::mat3 invInertiaWorld = computeWorldInvInertia(s);
    s.angularVel += invInertiaWorld * torque;
}

//----------------------------------------------------------
// CONTATO C CHAO
//----------------------------------------------------------
static void solveGroundContact(Shape& s, const glm::vec3& cp, float dt)
{
    //if (s.isStatic ) return;
    //printf(" >> <<")h
    glm::vec3 n(0, 1, 0);
    float penetration = -cp.y;

    if (glm::length2(s.vel) > WAKE_SPEEDVEL ||
        glm::length2(s.angularVel) > WAKE_SPEEDANGVEL)
    {
        wake(s);
    }


    // -------------------------------------------------
    //  Sem penetracao  ignora
    // -------------------------------------------------
    if (penetration <= 0.0f)
        return;

    // -------------------------------------------------
    //  NAO acordar por ruido
    // -------------------------------------------------
    //const float WAKE_SPEED2 = 0.0025f; // 5cm/s
    if (isEffectivelyStable(s))
    {
        //printf(" -- l 460 wake -- ");
        wake(s);
    }

    // -------------------------------------------------
    // Correcao posicional (Baumgarte estavel)
    // -------------------------------------------------
    const float slop = 0.001f;
    const float percent = 0.6f;
    const float maxCorrection = 0.1f;

    float correction =
        std::min(maxCorrection,
            std::max(penetration - slop, 0.0f) * percent);

    s.pos.y += correction;
    /*
    glm::vec3 cpUpdated = glm::vec3(s.pos.x, 0.0f, s.pos.z);
    glm::vec3 r = cpUpdated - s.pos;
   */

    // -------------------------------------------------
    // 4. Velocidade no ponto de contato
    // -------------------------------------------------
    glm::vec3 r = cp - s.pos;
    glm::vec3 vcp = s.vel + glm::cross(s.angularVel, r);
    float vn = glm::dot(vcp, n);
    /*
    if (s.isGrounded && std::abs(vn) < 0.01f)
    {
        printf(" shape : %d | grounded abs < 0.01f", s.type);
        s.vel.y = 0.0f;
        
        return;
    }
    */
    // Se j est se afastando ou quase parado → nAo resolver
    if (vn >= -1e-4f)
        return;

    float invM = getInvMass(s);
    glm::mat3 invInertiaWorld = computeWorldInvInertia(s);

    float denom = invM + glm::dot(invInertiaWorld * glm::cross(r, n), glm::cross(r, n));

    if (denom < 1e-8f)
        return;

    // -------------------------------------------------
    // 5. Impulso normal
    // -------------------------------------------------
    float jn = -(1.0f + restitutionGround) * vn / denom;

    // Ignora impulso microscópico
    if (std::abs(jn) > 1e-6f)
    {
        //printf(" shape id: %d -> abs jn: %lf\n", s.id, jn);
        applyImpulse(s, jn * n, cp);
    }
    // -------------------------------------------------
    // 6. Fricao Coulomb correta
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

            if (std::abs(jt) > 1e-6f) {
                //printf(" shape id: %d -> jt val: %lf", s.id, jt);
                applyImpulse(s, jt * t, cp);
               
            }
        }
    }

    // -------------------------------------------------
    // 7. Damping leve (sem destruir energia real)
    // -------------------------------------------------
    const float linearDamping = 0.98f;
    const float angularDamping = 0.98f;

    s.geometryDirty = true;

    s.vel *= linearDamping;
    s.angularVel *= angularDamping;
    
}
static bool isCOMInsideBoxTopFace(const Shape& box, const Shape& obj)
{
    // eixo local da box
    glm::mat3 R = glm::mat3_cast(box.rot);

    glm::vec3 local = glm::transpose(R) * (obj.pos - box.pos);

    glm::vec3 half = box.scale * 0.5f;

    // testa projeao em X e Z da face superior
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
static bool findPyramidContactFaceCentroid(
    const Shape& pyr,
    const glm::vec3& contactNormal,
    glm::vec3& outCentroid,
    glm::vec3& outFaceNormal)
{
    const auto& verts = pyr.geometry.vertices;

    // esperamos 5 vértices: 0-3 base, 4 apex
    if (verts.size() < 5)
        return false;

    // faces triangulares:
    // base = 2 triângulos
    // 4 faces laterais
    struct Tri { int a, b, c; };

    Tri faces[6] = {
        {0,1,2}, {0,2,3}, // base
        {0,1,4},
        {1,2,4},
        {2,3,4},
        {3,0,4}
    };

    float bestDot = -FLT_MAX;
    int bestIdx = -1;
    glm::vec3 bestNormal(0.0f);

    // contactNormal aponta de suporte -> objeto
    // queremos a face cuja normal aponte aproximadamente contra contactNormal
    glm::vec3 wanted = -glm::normalize(contactNormal);

    for (int i = 0; i < 6; ++i)
    {
        const glm::vec3& v0 = verts[faces[i].a];
        const glm::vec3& v1 = verts[faces[i].b];
        const glm::vec3& v2 = verts[faces[i].c];

        glm::vec3 n = glm::cross(v1 - v0, v2 - v0);
        float len2 = glm::length2(n);
        if (len2 < 1e-8f)
            continue;

        n = glm::normalize(n);

        // garantir normal apontando para fora
        // usamos centro da pirâmide como referência
        if (glm::dot(n, pyr.pos - v0) > 0.0f)
            n = -n;

        float d = glm::dot(n, wanted);

        if (d > bestDot)
        {
            bestDot = d;
            bestIdx = i;
            bestNormal = n;
        }
    }

    if (bestIdx == -1)
        return false;

    const glm::vec3& v0 = verts[faces[bestIdx].a];
    const glm::vec3& v1 = verts[faces[bestIdx].b];
    const glm::vec3& v2 = verts[faces[bestIdx].c];

    outCentroid = (v0 + v1 + v2) / 3.0f;
    outFaceNormal = bestNormal;

    return true;
}
static void handleGroundContact(Shape& s, float dt)
{
    bool wasGrounded = s.isGrounded;

    // Garantir world geometry atualizada
    if (s.geometryDirty)
    {
        updateShapeWorldGeometry(s);
        s.geometryDirty = false;
    }

    const float stableTimeRequired = 0.5f;
    const float posEps = 0.1f;      // 1 centímetro para posição
    const float rotEps = 0.0175f;
    const float GROUND_KEEP = 0.03f;   // 3 cm tolerância para manter
    const float GROUND_GAIN = 0.01f;

    float minY = FLT_MAX;
    glm::vec3 support(0.0f);

    if (s.type == ShapeType::Sphere)
    {
        float r = s.scale.x * s.radius;
        minY = s.pos.y - r;
        support = glm::vec3(s.pos.x, minY, s.pos.z);
    }
    else if (s.geometry.vertexCount > 0)
    {
        for (int i = 0; i < s.geometry.vertexCount; ++i)
        {
            const glm::vec3& v = s.geometry.vertices[i];

            if (v.y < minY)
            {
                minY = v.y;
                support = v;
            }
        }
    }
    float threshold = wasGrounded ? GROUND_KEEP : GROUND_GAIN;
    bool touching = (minY <= threshold);

    float posDiff = glm::length(s.pos - s.lastGroundedPos);
    float rotDiff = quatAngularDistance(s.lastGroundedRot, s.rot);
    bool abc = posDiff < posEps;
    bool test = posDiff < posEps && rotDiff < rotEps;
    printf(" abc -> (%d)  || ", abc);
    printf(" (%d) ", test);
    
    if (test)
    {
        //testar se aumenta mesmo
        s.groundedStableTimer += dt;
    }

    else if (posDiff > posEps * 3.0f || rotDiff > rotEps * 3.0f)
    {
        //printf(" -- l 710 groundedStable Timer reset -- ");
        //testar se zera mesmo com esse x3
        s.groundedStableTimer = 0.0f;
        s.lastGroundedPos = s.pos;
        s.lastGroundedRot = s.rot;
    }

    /*
    if (s.isGrounded)
    {
        glm::vec3 groundNormal(0, 1, 0);
        glm::vec3 up = glm::normalize(glm::mat3_cast(s.rot)[1]);

		const float dotVal = glm::dot(up, groundNormal);
        const float ALIGN_EPS = 0.995f;

        if (dotVal > ALIGN_EPS && !wasGrounded)
        {
            glm::vec3 right = glm::normalize(glm::mat3_cast(s.rot)[0]);
            glm::vec3 forward = glm::cross(groundNormal, right);
            right = glm::cross(forward, groundNormal);

            glm::mat3 basis;
            basis[0] = glm::normalize(right);
            basis[1] = groundNormal;
            basis[2] = glm::normalize(forward);

            s.rot = glm::quat_cast(basis);
            s.vel = glm::vec3(0.0f);
            s.angularVel = glm::vec3(0.0f);
        }
    }
    */
    
    /*
    
    if (touching)
    {
        s.isGrounded = true;
        
        if (isEffectivelyStable(s)) {
            s.isSleeping = true;
            
        }
    }
    else 
    {
        s.groundedStableTimer = 0.0f;
        if (minY > GROUND_KEEP) {
            
            s.isGrounded = false;
            s.isStatic = false;
            s.isSleeping = false;
        }
    }
    */
    if (touching) {
        //s.isGrounded = true;
        //if (isEffectivelyStable(s)) {
            //s.isSleeping = true;
          //  s.groundedStableTimer += dt;
            //return;
        //}
        
        //s.groundedStableTimer = 0.0f;
        solveGroundContact(s, support, dt);
        

    }
    
    else {
        //printf(" -- l 778 groundedstabletimer reset -- ");
        s.isGrounded = false;
        s.groundedStableTimer = 0.0f;

    }
    

        
}

static bool isPointInsideBoxTopFace(const Shape& box, const glm::vec3& worldPoint)
{
    glm::mat3 R = glm::mat3_cast(box.rot);
    glm::vec3 local = glm::transpose(R) * (worldPoint - box.pos); // point em espaço local da box
    glm::vec3 half = box.scale * 0.5f;

    // checa projeao X/Z dentro dos half-extents da face superior
    // nAo exigimos que y esteja exatamente no topo; apenas que projete sobre a rea da face 1e-6f
    return (std::abs(local.x) <= half.x + 1e-6f && std::abs(local.z) <= half.z);
}



inline bool guessNormalAndPoint(const glm::vec3& a, const glm::vec3& b,
    glm::vec3& outSupportNormal, glm::vec3& outContactPoint, const Shape& s)
{
    // heuristica: um dos argumentos provavelmente e uma normal unitria (~1.0 length)
    // e o outro e um ponto (distancia razovel ao centro do shape).
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

    // obtem normais das faces em world
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

    // se j praticamente alinhado, apenas corrige penetracoes
    glm::vec3 chosenFrom = glm::normalize(faceNormals[bestIdx]);
    const auto& verts = s.geometry.vertices;
    if (bestDot > 0.9995f) {
        // apenas empurra para evitar interpenetraao com o plano definido por contactPoint+supportNormal
        //std::vector<glm::vec3> verts; 
        //BoxVerts verts;
        //getBoxVertices(s, verts);
        
        float minDist = FLT_MAX;
        for (auto& v : verts) {
            float d = glm::dot(v - contactPoint, supportNormal);
            minDist = std::min(minDist, d);
        }
        if (minDist < 0.0f) s.pos -= supportNormal * minDist;
        return;
    }

    // calcula rotaao que leva a face escolhida -> supportNormal
    glm::quat q = glm::rotation(chosenFrom, supportNormal);
    q = glm::normalize(q);

    // rotaciona em torno do ponto de contato (mantem contato onde est)
    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);
    s.pos = contactPoint - rotatedOffset;

    s.rot = glm::normalize(q * s.rot);

    // corrige penetraao pequena após rotaao (garante que vertices nAo fiquem "dentro" do suporte)
    
    float minDist = FLT_MAX;
    for (auto& v : verts) {
        float d = glm::dot(v - contactPoint, supportNormal);
        minDist = std::min(minDist, d);
    }
    if (minDist < 0.0f)
        s.pos -= supportNormal * minDist;
    s.geometryDirty = true;

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
    s.geometryDirty = true;
    s.angularVel *= 0.2f;
    s.vel *= 0.4f;
}
inline void snapPyramidToSurface(Shape& s,
    const glm::vec3& supportNormal,
    const glm::vec3& contactPoint)
{
    if (s.isStatic) return;

    const auto& verts = s.geometry.vertices;
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
    s.geometryDirty = true;

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
    // Obtem vertices da piramide
    //--------------------------------------------------
    const auto& verts = s.geometry.vertices;
    if (verts.size() < 5) return;

    //--------------------------------------------------
    // Faces da piramide (base + 4 laterais)
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
    // Se j praticamente alinhado → só corrige penetraao
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
    // Rotaao da face escolhida → supportNormal
    //--------------------------------------------------
    glm::quat q = glm::rotation(bestNormal, supportNormal);
    q = glm::normalize(q);

    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);

    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);

    //--------------------------------------------------
    // Corrige penetraao residual pós-rotaao
    //--------------------------------------------------
    verts.empty();
    //verts = s.geometry.vertices;

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

    // trava rotaao residual pequena
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
    float minDist = FLT_MAX;

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
    else
    {
        const auto& verts = s.geometry.vertices;

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

    else
    {
        float minY = FLT_MAX;
        const auto& verts = s.geometry.vertices;
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
// SOLUaO DE CONTATO ENTRE CORPOS (CORRIGIDA)
//----------------------------------------------------------
// --- helpers para estabilidade entre box <-> pyramid ---------------------------------

static void solveContact(Contact& c, float dt) {
    Shape& A = *c.a; Shape& B = *c.b; 
    if (A.isStatic && B.isStatic) 
        return; 
    if (c.penetration > 0.01f)
    {
        wake(A);
        wake(B);
    }
    /*
    // --- Novidade: cheque de "support" estvel entre box <-> pyramid ---- 
    auto isStable = [](const Shape& s) { 
        bool stable = s.isStatic || s.isSleeping || s.isGrounded; 
        return stable; 
        }; */
    glm::vec3 up(0.0f, 1.0f, 0.0f); 
    glm::vec3 n = glm::normalize(c.normal); 
    if (glm::length2(n) < 1e-8f) 
        return; // normal aponta de A -> B (convenao seu Contact) 
    
    bool A_supports_B = isEffectivelyStable(A) && glm::dot(n, up) > 0.01f;
    
    bool B_supports_A = isEffectivelyStable(B) && glm::dot(-n, up) > 0.5f; 
    // Se A e box e B e piramide: verificar centróide da face de B 
    if (A_supports_B && A.type == ShapeType::Rect && B.type == ShapeType::Rect) { 
        glm::vec3 faceCentroid, faceNormal; 
        if (findPyramidContactFaceCentroid(B, n, faceCentroid, faceNormal)) { // se centroide da face de piramide projeta sobre face superior da box -> grounded 
            if (isPointInsideBoxTopFace(A, faceCentroid)) { //printf("ponto rect rect "); //A.isGrounded = true; //A.isSleeping = true; 
                /*B.isGrounded = true;
                B.isSleeping = true; // atualiza histórico de grounded para hysteresis/stable detection 
                B.lastGroundedPos = B.pos; 
                B.lastGroundedRot = B.rot; */
                /*
                * B.isGrounded = true;
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
                */
                if (isEffectivelyStable(B)) {
                    B.isGrounded = true;
                    A.isGrounded = true;

                    B.sleepTimer += dt;
                    A.sleepTimer += dt;

                }
                B.groundedStableTimer += dt;
                
            } 
            else { //printf("permite tombar/rotacionar rect"); // permite tombar/rotacionar (nAo marcar como grounded) 
                B.isGrounded = false; 
                A.isGrounded = true; //A.isSleeping = true; 
        } 
        } 
        else { //printf("sem face detectada"); // sem face detectada, fallback: comportamento padrAo (nAo forçar grounded) 
            B.isGrounded = false; 
        } 
    } if (A_supports_B && A.type == ShapeType::Rect && B.type == ShapeType::Pyramid ) {
        glm::vec3 faceCentroid, faceNormal; 
        if (findPyramidContactFaceCentroid(B, n, faceCentroid, faceNormal)) { // se centróide da face de piramide projeta sobre face superior da box -> grounded 
            printf(" ----- ");
            if (isPointInsideBoxTopFace(A, faceCentroid)) { //A.isGrounded = true; //A.isSleeping = true; 
                /*B.isGrounded = true;
                B.isSleeping = true; // atualiza histórico de grounded para hysteresis/stable detection 
                B.lastGroundedPos = B.pos; 
                B.lastGroundedRot = B.rot; 

                */
                printf(" +++++ ");
                A.isGrounded = true;
                A.isSleeping = true;
                if (isEffectivelyStable(B)) {
                    B.isGrounded = true;
                    

                    B.sleepTimer += dt;
                    A.sleepTimer += dt;

                }
                B.groundedStableTimer += dt;

            }
            else { //printf("permite tombar/rotacionar rect"); // permite tombar/rotacionar (nAo marcar como grounded) 
                B.isGrounded = false;
                A.isGrounded = true; //A.isSleeping = true; 
            }
        } else { //printf("sem face detectada"); // sem face detectada, fallback: comportamento padrAo (nAo forçar grounded) 
            B.isGrounded = false; 
        }
    } // Se B e box e A e piramide (o caso invertido): mesma lógica simetrica 
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
    } else { // casos gerais: mantemos a heuristica anterior (se o suporte for "estvel", marca grounded) 
            if (A_supports_B) B.isGrounded = true; 
            if (B_supports_A) A.isGrounded = true; 
    } 
    float invMA = getInvMass(A); 
    float invMB = getInvMass(B); 
    glm::mat3 invIA = computeWorldInvInertia(A); 
    glm::mat3 invIB = computeWorldInvInertia(B); 
    glm::vec3 rA = c.point - A.pos; 
    glm::vec3 rB = c.point - B.pos; 
    glm::vec3 vA = A.vel + glm::cross(A.angularVel, rA); 
    glm::vec3 vB = B.vel + glm::cross(B.angularVel, rB); 
    glm::vec3 rv = vB - vA; float vn = glm::dot(rv, n); // Correao posicional (projeao) 
            { const float slop = 0.001f; 
            const float percent = 0.7f; 
            float pen = std::max(c.penetration - slop, 0.0f); 
            float denom = invMA + invMB; 
            if (denom > 0.0f) { 
                glm::vec3 corr = (pen * percent / denom) * n; 
                if (!A.isStatic) A.pos -= corr * invMA; 
                if (!B.isStatic) B.pos += corr * invMB; 
            } 
    } // se corpos se separando ao longo da normal, nAo aplica impulso normal 
            if (vn > 0.0f) return; 
            float bounceThreshold = 0.2f; // 20 cm/s

            float e = (std::abs(vn) > bounceThreshold)
                ? restitutionGround
                : 0.0f;
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
            } // --- Fricao tangencial (Coulomb) --- 
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
static bool spherePyramid(Shape& sphere, Shape& pyr, Contact& c)
{
    float r = sphereRadius(sphere);

    const auto& faces = pyr.geometry.faces;
    int faceCount = pyr.geometry.faceCount;

    float maxSeparation = -FLT_MAX;
    int maxIdx = -1;

    glm::vec3 sphereCenter = sphere.pos;

    for (int i = 0; i < faceCount; ++i)
    {
        float d = glm::dot(faces[i].n, sphereCenter) + faces[i].d;

        if (d > r)
            return false;

        if (d > maxSeparation)
        {
            maxSeparation = d;
            maxIdx = i;
        }
    }

    if (maxIdx == -1)
        return false;

    const Face& refFace = faces[maxIdx];

    c.a = &pyr;
    c.b = &sphere;
    c.normal = refFace.n;
    c.penetration = r - maxSeparation;
    c.point = sphereCenter - refFace.n * r;

    return true;
}
//----------------------------------------------------------
// DETECaO DE COLISÕES
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
    /*
    BoxVerts vertsA;
    BoxVerts vertsB;

    getBoxVertices(a, vertsA);
    getBoxVertices(b, vertsB);*/

    const auto& vertsA = a.geometry.vertices;
    const auto& vertsB = b.geometry.vertices;



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
    /*
    BoxVerts vertsBox;
    PyramidVerts vertsPyr;

    getBoxVertices(box, vertsBox);
    getPyramidVertices(pyr, vertsPyr);*/

    const auto& vertsBox = box.geometry.vertices;
    const auto& vertsPyr = pyr.geometry.vertices;


    glm::mat3 R = glm::mat3_cast(box.rot);

    const auto& faces = pyr.geometry.faces;
    int faceCount = pyr.geometry.faceCount;


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
        if (!testAxis(faces[i].n)) return false;

    // cross products
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 5; ++j)
            if (!testAxis(glm::cross(R[i], faces[j].n)))
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
inline void sleepCheck(Shape& s) {
    const float SLEEP_TIME = 0.6f;
    if (s.sleepTimer >= SLEEP_TIME || s.groundedStableTimer >= SLEEP_TIME)
    {

        s.vel = glm::vec3(0.0f);
        s.angularVel = glm::vec3(0.0f);
        s.isGrounded = true;
        s.isSleeping = true;
        s.isStatic = true;
        return;
    }
    s.isSleeping = false;
    s.sleepTimer = 0.0f;
}
inline void angularTestAndFix(Shape& s) {

    // =============================
    // TESTE ANGULAR (quase paralelo ao chAo)
    // =============================


    float lin2 = glm::length2(s.vel);
    float ang2 = glm::length2(s.angularVel);

    glm::vec3 groundNormal(0, 1, 0);
    glm::vec3 supportPoint, supportNormal;

    bool hasSupport = findLocalSupport(s, supportPoint, supportNormal);

    bool isNearlyFlat = false;

    if (hasSupport)
    {
        float dotVal = glm::dot(glm::normalize(supportNormal), groundNormal);

        // 0.95 ≈ ate ~18 graus de inclinaao
        const float ALIGN_THRESHOLD = 0.95f;

        isNearlyFlat = dotVal > ALIGN_THRESHOLD && ang2 < 4e-4f;
    }

    if (isNearlyFlat)
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
}
inline void sleepFilter(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging) {
        return;
    }


    float lin2 = glm::length2(s.vel);
    float ang2 = glm::length2(s.angularVel);

    // =============================
    // CONFIGURAaO
    // =============================

    const float LIN_SLEEP_EPS = 0.05f;
    const float ANG_SLEEP_EPS = 0.05f;     // tolerancia angular
    const float HARD_RESET_SPEED = 0.05f; // 5 cm/s real
    const float HARD_RESET_SPEED2 = HARD_RESET_SPEED * HARD_RESET_SPEED;  // movimento real
    
    const float MIN_PARALLEL = 0.995f;



    // =============================
    // Se nAo est grounded, nunca dorme
    // =============================

    if (!s.isGrounded)
    {
        //printf(" -- l 1828 sleep timer reset --");
        //printf(" **nearlyflat** ");
        s.sleepTimer = 0.0f;
    //sleeping = false
        return;
    }

    bool slowEnough =
        lin2 < LIN_SLEEP_EPS * LIN_SLEEP_EPS &&
        ang2 < ANG_SLEEP_EPS * ANG_SLEEP_EPS;
    //printf(" lin2 : %lf, ang2 : %lf ", lin2, ang2);
    //printf(" %d ", slowEnough);
    if (slowEnough && s.isGrounded)
    {
        // força zero absoluto para matar jitter
        s.vel = glm::vec3(0.0f);
        s.angularVel = glm::vec3(0.0f);

        s.sleepTimer += dt;
    }
    else
    {
        //hprintf(" -- l 1850 sleep timer -- ");
        s.sleepTimer = 0.0f;
        // Só resetar se movimento for REAL
        if (!s.isGrounded &&
            (lin2 > HARD_RESET_SPEED2 || ang2 > HARD_RESET_SPEED2))
        {
            //printf(" ** hard reset ** ");
            //s.sleepTimer = 0.0f;
            s.isSleeping = false;
        }
    }
}
void updatePhysics(Scene& scene, float dt)
{

    if (dt <= 0.0f) return;



    auto& spheres = scene.spheresRef();
    auto& boxes = scene.rectsRef();
    auto& pyramids = scene.pyramidsRef();
    /*

    for (auto& s : spheres)  wakeIfThresholdPassed(s);
    for (auto& b : boxes)    wakeIfThresholdPassed(b);
    for (auto& p : pyramids) wakeIfThresholdPassed(p);*/
    // -------------------------------------------------
    // 8) Sleep check
    // -------------------------------------------------
    for (auto& s : spheres)  sleepCheck(s);
    for (auto& b : boxes)    sleepCheck(b);
    for (auto& p : pyramids) sleepCheck(p);

    // -------------------------------------------------
    // 1) Integrar gravidade velocidade (forças -> altera vel)
    // -------------------------------------------------
    for (auto& s : spheres)  integrateGravity(s, dt);
    for (auto& b : boxes)    integrateGravity(b, dt);
    for (auto& p : pyramids) integrateGravity(p, dt);

    /*
    for (auto& s : spheres)  integrateRotation(s, dt);
    for (auto& b : boxes)    integrateRotation(b, dt);
    for (auto& p : pyramids) integrateRotation(p, dt);

    */
    // -------------------------------------------------
    // 7) Integrar rotação (APÓS resolver impulsos)
    // -------------------------------------------------
    

    // -------------------------------------------------
    // 8) Sleep check
    // -------------------------------------------------
    for (auto& s : spheres)  sleepFilter(s, dt);
    for (auto& b : boxes)    sleepFilter(b, dt);
    for (auto& p : pyramids) sleepFilter(p, dt);

    // -------------------------------------------------
    // 2) Integrar posição (pos += vel * dt) — marca geometria dirty
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
    // 3) Atualizar geometria world (somente dirty)
    //    — garante que todos os vértices/faces em world estão prontos
    // -------------------------------------------------
    auto updateGeom = [](auto& container)
        {
            for (auto& s : container)
            {
                if (s.geometryDirty)
                {
                    updateShapeWorldGeometry(s);
                    s.geometryDirty = false;
                }
            }
        };

    updateGeom(spheres);
    updateGeom(boxes);
    updateGeom(pyramids);

    // -------------------------------------------------
    // 4) Contato com chão (após posições atualizadas e geometria em world)
    // -------------------------------------------------
    for (auto& s : spheres)  handleGroundContact(s, dt);
    for (auto& b : boxes)    handleGroundContact(b, dt);
    for (auto& p : pyramids) handleGroundContact(p, dt);

    // -------------------------------------------------
    // 5) Broadphase AABB (usando geometria atualizada)
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
    // 6) Narrowphase & resolve
    // -------------------------------------------------
    Contact c;

    // esfera-esfera
    for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = i + 1; j < spheres.size(); ++j)
        {
            if (!aabbOverlap(sphereAABB[i], sphereAABB[j])) continue;
            if (sphereSphere(spheres[i], spheres[j], c))
                solveContact(c, dt);
        }

    // esfera-box
    for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = 0; j < boxes.size(); ++j)
        {
            if (!aabbOverlap(sphereAABB[i], boxAABB[j])) continue;
            if (sphereBox(spheres[i], boxes[j], c))
                solveContact(c, dt);
        }

    // box-box
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = i + 1; j < boxes.size(); ++j)
        {
            if (!aabbOverlap(boxAABB[i], boxAABB[j])) continue;
            if (boxBox(boxes[i], boxes[j], c))
                solveContact(c, dt);
        }

    // box-piramide
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = 0; j < pyramids.size(); ++j)
        {
            if (!aabbOverlap(boxAABB[i], pyramidAABB[j])) continue;
            if (boxPyramid(boxes[i], pyramids[j], c))
                solveContact(c, dt);
        }

    // esfera-piramide
    for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = 0; j < pyramids.size(); ++j)
        {
            if (!aabbOverlap(sphereAABB[i], pyramidAABB[j])) continue;
            if (spherePyramid(spheres[i], pyramids[j], c))
                solveContact(c, dt);
        }

   


}