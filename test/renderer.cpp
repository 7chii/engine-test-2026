#include "renderer.h"

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>
#include <iostream>

#include "scene.h"

// Renderer implementation: compiles simple color shader and creates VAOs used by scene::drawScene.
// Keeps state so main/app can be minimal.

struct Renderer::Impl {
    unsigned int shaderProgram = 0;
    unsigned int shadowShaderProgram = 0;
    unsigned int floorProgram = 0;
    unsigned int vaoGrid = 0;
    unsigned int vaoFloor = 0;
    unsigned int vaoPyramid = 0;
    unsigned int vaoRect = 0;
    unsigned int vaoSphere = 0;
    unsigned int sphereIndexCount = 0;
    glm::vec3 prevCameraPos = glm::vec3(0.0f);
    float lastMouseX = 600.0f, lastMouseY = 400.0f;
};

Renderer::Renderer() : _p(new Impl()) {}
Renderer::~Renderer() { delete _p; }

static unsigned int compileShader(unsigned int type, const char* src) {
    unsigned int s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    return s;
}

static unsigned int linkProgram(unsigned int vs, unsigned int fs) {
    unsigned int prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// ===== FORWARD DECLARATIONS =====
static void drawSceneWithShadow(
    const Scene& scene,
    unsigned int shadowShaderProgram,
    unsigned int vaoPyramid,
    unsigned int vaoRect,
    unsigned int vaoSphere,
    int sphereIndexCount,
    const glm::mat4& view,
    const glm::mat4& projection
);

static void drawOutlines(
    const Scene& scene,
    unsigned int shadowShaderProgram,
    unsigned int vaoPyramid,
    unsigned int vaoRect,
    unsigned int vaoSphere,
    int sphereIndexCount,
    const glm::mat4& view,
    const glm::mat4& projection
);

void Renderer::init() {
    
    const char* vShader = R"(
#version 460 core
layout (location = 0) in vec3 aPos;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";
    const char* fShader = R"(
#version 460 core
out vec4 FragColor;
uniform vec3 color;
void main() {
    FragColor = vec4(color, 1.0);
}
)";

    unsigned int vs = compileShader(GL_VERTEX_SHADER, vShader);
    unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fShader);
    _p->shaderProgram = linkProgram(vs, fs);

    
    const char* shadowVS = R"(
#version 460 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out VS_OUT {
    vec3 worldPos;
    flat int primitiveID;
} vs_out;

void main() {
    vs_out.worldPos = (model * vec4(aPos, 1.0)).xyz;
    vs_out.primitiveID = gl_VertexID / 3;  // 3 vertices per triangle
    gl_Position = projection * view * vec4(vs_out.worldPos, 1.0);
}
)";

    const char* shadowFS = R"(
#version 460 core

in VS_OUT {
    vec3 worldPos;
    flat int primitiveID;
} fs_in;

out vec4 FragColor;

uniform vec3 baseColor;
uniform vec3 lightDir = normalize(vec3(0.3, 1.0, 0.5));  // Luz vindo de cima-diagonal

void main() {
    // Compute face normal using derivatives
    vec3 dx = dFdx(fs_in.worldPos);
    vec3 dy = dFdy(fs_in.worldPos);
    vec3 normal = normalize(cross(dx, dy));
    
    // Shade baseado no angulo entre normal e luz
    float shade = dot(normal, lightDir);
    shade = clamp(shade, 0.3, 1.0);  // Keep minimum brightness
    
    vec3 shadedColor = baseColor * shade;
    FragColor = vec4(shadedColor, 1.0);
}
)";

    unsigned int svs = compileShader(GL_VERTEX_SHADER, shadowVS);
    unsigned int sfs = compileShader(GL_FRAGMENT_SHADER, shadowFS);
    _p->shadowShaderProgram = linkProgram(svs, sfs);

  
    const char* floorVS = R"(
#version 460 core
layout (location = 0) in vec3 aPos;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec3 worldPos;
void main() {
    worldPos = (model * vec4(aPos, 1.0)).xyz;
    gl_Position = projection * view * vec4(worldPos, 1.0);
}
)";
    const char* floorFS = R"(
#version 460 core
in vec3 worldPos;
out vec4 FragColor;
uniform vec3 colorA;
uniform vec3 colorB;
uniform float scale;
uniform float alpha;
void main() {
    float cx = floor(worldPos.x * scale);
    float cz = floor(worldPos.z * scale);
    float v = mod(cx + cz, 2.0);
    vec3 col = (v < 0.5) ? colorA : colorB;
    FragColor = vec4(col, alpha);
}
)";
    unsigned int fvs = compileShader(GL_VERTEX_SHADER, floorVS);
    unsigned int ffs = compileShader(GL_FRAGMENT_SHADER, floorFS);
    _p->floorProgram = linkProgram(fvs, ffs);

    // create grid geometry (lines)
    std::vector<float> grid;
    for (int i = -50; i <= 50; i++) {
        grid.insert(grid.end(), { (float)i,0,-50, (float)i,0,50, -50,0,(float)i, 50,0,(float)i });
    }

    auto createVAOFromData = [](const float* data, size_t size)->unsigned int {
        unsigned int vao, vbo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        return vao;
        };

    float pyrVerts[] = {
       
        // frente
        0, 0.5, 0,   -0.5,-0.5, 0.5,   0.5,-0.5, 0.5,
        // direita
        0, 0.5, 0,    0.5,-0.5, 0.5,   0.5,-0.5,-0.5,
        // bunda
        0, 0.5, 0,    0.5,-0.5,-0.5,  -0.5,-0.5,-0.5,
        // esqu
        0, 0.5, 0,   -0.5,-0.5,-0.5,  -0.5,-0.5, 0.5,

        // base
        -0.5,-0.5,-0.5,   0.5,-0.5,-0.5,   0.5,-0.5, 0.5,
         0.5,-0.5, 0.5,  -0.5,-0.5, 0.5,  -0.5,-0.5,-0.5
    };


    // full box 
    float rectVerts[] = {
        // front
        -0.5f,-0.5f,-0.5f,  0.5f,-0.5f,-0.5f,  0.5f,0.5f,-0.5f,
         0.5f,0.5f,-0.5f, -0.5f,0.5f,-0.5f, -0.5f,-0.5f,-0.5f,
         // back
         -0.5f,-0.5f,0.5f,  0.5f,-0.5f,0.5f,  0.5f,0.5f,0.5f,
          0.5f,0.5f,0.5f,  -0.5f,0.5f,0.5f,  -0.5f,-0.5f,0.5f,
          // left
          -0.5f,0.5f,0.5f, -0.5f,0.5f,-0.5f, -0.5f,-0.5f,-0.5f,
          -0.5f,-0.5f,-0.5f, -0.5f,-0.5f,0.5f, -0.5f,0.5f,0.5f,
          // right
           0.5f,0.5f,0.5f,  0.5f,0.5f,-0.5f,  0.5f,-0.5f,-0.5f,
           0.5f,-0.5f,-0.5f, 0.5f,-0.5f,0.5f,  0.5f,0.5f,0.5f,
           // bottom
           -0.5f,-0.5f,-0.5f, 0.5f,-0.5f,-0.5f, 0.5f,-0.5f,0.5f,
            0.5f,-0.5f,0.5f, -0.5f,-0.5f,0.5f, -0.5f,-0.5f,-0.5f,
            // top
            -0.5f,0.5f,-0.5f,  0.5f,0.5f,-0.5f,  0.5f,0.5f,0.5f,
             0.5f,0.5f,0.5f, -0.5f,0.5f,0.5f, -0.5f,0.5f,-0.5f
    };

    _p->vaoGrid = createVAOFromData(grid.data(), grid.size() * sizeof(float));
    _p->vaoPyramid = createVAOFromData(pyrVerts, sizeof(pyrVerts));
    _p->vaoRect = createVAOFromData(rectVerts, sizeof(rectVerts));

    // floor quad
    float floorVerts[] = {
        -50.0f, 0.0f, -50.0f,
         50.0f, 0.0f, -50.0f,
         50.0f, 0.0f,  50.0f,
        -50.0f, 0.0f, -50.0f,
         50.0f, 0.0f,  50.0f,
        -50.0f, 0.0f,  50.0f
    };
    _p->vaoFloor = createVAOFromData(floorVerts, sizeof(floorVerts));

    // sphere mesh
    std::vector<float> sphereVerts;
    std::vector<unsigned int> sphereIndices;
    const int sectors = 20, stacks = 20;
    for (int i = 0; i <= stacks; ++i) {
        float stackAngle = glm::pi<float>() / 2 - i * glm::pi<float>() / stacks;
        float xy = cosf(stackAngle);
        float z = sinf(stackAngle);
        for (int j = 0; j <= sectors; ++j) {
            float sectorAngle = j * 2 * glm::pi<float>() / sectors;
            sphereVerts.push_back(xy * cosf(sectorAngle));
            sphereVerts.push_back(xy * sinf(sectorAngle));
            sphereVerts.push_back(z);
        }
    }
    for (int i = 0; i < stacks; ++i) {
        int k1 = i * (sectors + 1);
        int k2 = k1 + sectors + 1;
        for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
            if (i != 0) { sphereIndices.push_back(k1); sphereIndices.push_back(k2); sphereIndices.push_back(k1 + 1); }
            if (i != (stacks - 1)) { sphereIndices.push_back(k1 + 1); sphereIndices.push_back(k2); sphereIndices.push_back(k2 + 1); }
        }
    }

    unsigned int vaoS, vboS, eboS;
    glGenVertexArrays(1, &vaoS);
    glGenBuffers(1, &vboS);
    glGenBuffers(1, &eboS);
    glBindVertexArray(vaoS);
    glBindBuffer(GL_ARRAY_BUFFER, vboS);
    glBufferData(GL_ARRAY_BUFFER, sphereVerts.size() * sizeof(float), sphereVerts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboS);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndices.size() * sizeof(unsigned int), sphereIndices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    _p->vaoSphere = vaoS;
    _p->sphereIndexCount = (int)sphereIndices.size();
}

void Renderer::draw(const Scene& scene, const Camera& cam) {
    // prepare matrix
    glm::mat4 view = glm::lookAt(cam.pos, cam.pos + cam.front, cam.up);
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), 1200.0f / 800.0f, 0.1f, 100.0f);

    // draw floor
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(_p->floorProgram);
    glUniformMatrix4fv(glGetUniformLocation(_p->floorProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(_p->floorProgram, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
    glm::mat4 model = glm::mat4(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(_p->floorProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(glGetUniformLocation(_p->floorProgram, "colorA"), 0.12f, 0.12f, 0.12f);
    glUniform3f(glGetUniformLocation(_p->floorProgram, "colorB"), 0.18f, 0.18f, 0.18f);
    glUniform1f(glGetUniformLocation(_p->floorProgram, "scale"), 0.5f);
    glUniform1f(glGetUniformLocation(_p->floorProgram, "alpha"), 0.6f);
    glBindVertexArray(_p->vaoFloor);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisable(GL_BLEND);

    // draw grid lines 
    glUseProgram(_p->shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(_p->shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(_p->shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
    glm::mat4 id = glm::mat4(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(_p->shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(id));
    glUniform3f(glGetUniformLocation(_p->shaderProgram, "color"), 0.6f, 0.6f, 0.6f);
    glBindVertexArray(_p->vaoGrid);
    glDrawArrays(GL_LINES, 0, 404);

    // draw scene objects with shading
    drawSceneWithShadow(scene, _p->shadowShaderProgram, _p->vaoPyramid, _p->vaoRect, _p->vaoSphere, (int)_p->sphereIndexCount, view, proj);

    // draw outlines on top (depois de tudo)
    drawOutlines(scene, _p->shadowShaderProgram, _p->vaoPyramid, _p->vaoRect, _p->vaoSphere, (int)_p->sphereIndexCount, view, proj);
}

//  objetos com sombreamento
static void drawSceneWithShadow(
    const Scene& scene,
    unsigned int shadowShaderProgram,
    unsigned int vaoPyramid,
    unsigned int vaoRect,
    unsigned int vaoSphere,
    int sphereIndexCount,
    const glm::mat4& view,
    const glm::mat4& projection
) {
    glUseProgram(shadowShaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    auto drawShape = [&](unsigned int vao, const Shape& s, int count, bool indexed, ShapeType t, int idx) {
        glm::mat4 m = glm::translate(glm::mat4(1.0f), s.pos);
        m = m * glm::mat4_cast(s.rot);
        m = glm::scale(m, s.scale);

        glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(m));
        glUniform3f(glGetUniformLocation(shadowShaderProgram, "baseColor"), 0.7f, 0.7f, 0.9f);

        glBindVertexArray(vao);
        if (indexed) glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, 0);
        else glDrawArrays(GL_TRIANGLES, 0, count);
        };

    const auto& ps = scene.pyramidsRef();
    const auto& rs = scene.rectsRef();
    const auto& ss = scene.spheresRef();

    for (int i = 0; i < (int)ps.size(); ++i)
        drawShape(vaoPyramid, ps[i], 18, false, ShapeType::Pyramid, i);


    for (int i = 0; i < (int)rs.size(); ++i)
        drawShape(vaoRect, rs[i], 36, false, ShapeType::Rect, i);

    for (int i = 0; i < (int)ss.size(); ++i)
        drawShape(vaoSphere, ss[i], sphereIndexCount, true, ShapeType::Sphere, i);
}

//  outlines 
static void drawOutlines(
    const Scene& scene,
    unsigned int shadowShaderProgram,
    unsigned int vaoPyramid,
    unsigned int vaoRect,
    unsigned int vaoSphere,
    int sphereIndexCount,
    const glm::mat4& view,
    const glm::mat4& projection
) {
    const Selection& sel = scene.selectionRef();

    if (sel.type == ShapeType::None) return;  

    glUseProgram(shadowShaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    
    glDisable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); 
    glLineWidth(2.0f);

    auto drawOutlineShape = [&](unsigned int vao, const Shape& s, int count, bool indexed, ShapeType t, int idx) {
        if (sel.type != t || sel.index != idx) return;  //so desenha se sel

        
        glm::mat4 om = glm::translate(glm::mat4(1.0f), s.pos);
        om = om * glm::mat4_cast(s.rot);
        om = glm::scale(om, s.scale);

        glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(om));
        glUniform3f(glGetUniformLocation(shadowShaderProgram, "baseColor"), 1.0f, 0.0f, 0.0f);  // vermei para outline

        glBindVertexArray(vao);
        if (indexed) glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, 0);
        else glDrawArrays(GL_TRIANGLES, 0, count);
        };

    const auto& ps = scene.pyramidsRef();
    const auto& rs = scene.rectsRef();
    const auto& ss = scene.spheresRef();

    for (int i = 0; i < (int)ps.size(); ++i)
        drawOutlineShape(vaoPyramid, ps[i], 18, false, ShapeType::Pyramid, i);

    for (int i = 0; i < (int)rs.size(); ++i)
        drawOutlineShape(vaoRect, rs[i], 36, false, ShapeType::Rect, i);

    for (int i = 0; i < (int)ss.size(); ++i)
        drawOutlineShape(vaoSphere, ss[i], sphereIndexCount, true, ShapeType::Sphere, i);

    // restaurar estado
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); 
    glLineWidth(1.0f); 
    glEnable(GL_DEPTH_TEST);
}

void Renderer::setPrevCameraPos(const glm::vec3& p) { _p->prevCameraPos = p; }
glm::vec3 Renderer::getPrevCameraPos() const { return _p->prevCameraPos; }

void Renderer::setLastMouse(float x, float y) { _p->lastMouseX = x; _p->lastMouseY = y; }
float Renderer::getLastX() const { return _p->lastMouseX; }
float Renderer::getLastY() const { return _p->lastMouseY; }