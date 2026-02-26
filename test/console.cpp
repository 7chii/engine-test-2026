#include "console.h"

#include <imgui.h>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include "physics.h"

#include "shape.h"

#include "scene.h"


static float extractMassFromTokens(const std::vector<std::string>& tok, float defaultMass) {
    for (size_t i = 0; i + 1 < tok.size(); ++i) {
        if (tok[i] == "-p" || tok[i] == "-P") {
            try { return std::stof(tok[i + 1]); }
            catch (...) { return defaultMass; }
        }
    }
    return defaultMass;
}

static std::vector<std::string> splitTokens(const std::string& s) {
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string t;
    while (iss >> t) out.push_back(t);
    return out;
}

static void processCommand(Scene& scene, const std::string& cmd) {
    auto tok = splitTokens(cmd);
    if (tok.empty()) return;

    auto& pyramids = scene.pyramidsRef();
    auto& rects = scene.rectsRef();
    auto& spheres = scene.spheresRef();

    if (tok.size() >= 5 && tok[0] == "add" && tok[1] == "pyramid") {
        float x = std::stof(tok[2]), y = std::stof(tok[3]), z = std::stof(tok[4]);
        float m = extractMassFromTokens(tok, 1.0f);
        Shape s; s.pos = glm::vec3(x, y, z); s.scale = glm::vec3(1.0f); s.mass = m;
        s.id = (int)scene.pyramidsRef().size();
        s.inertia = computeLocalInertiaTensor(s);
        s.invInertia = glm::inverse(s.inertia);
        pyramids.push_back(s);
        return;
    }
    if (tok.size() >= 5 && tok[0] == "add" && tok[1] == "sphere") {
        float x = std::stof(tok[2]), y = std::stof(tok[3]), z = std::stof(tok[4]);
        float m = extractMassFromTokens(tok, 1.0f);
        Shape s; s.pos = glm::vec3(x, y, z); s.scale = glm::vec3(1.0f); s.mass = m;
        s.id = (int)scene.spheresRef().size();
        s.inertia = computeLocalInertiaTensor(s);
        s.invInertia = glm::inverse(s.inertia);
        spheres.push_back(s);
        return;
    }
    if (tok.size() >= 9 && tok[0] == "add" && tok[1] == "rect") {
        float x = std::stof(tok[2]), y = std::stof(tok[3]), z = std::stof(tok[4]);
        int di = -1;
        for (size_t i = 0; i < tok.size(); ++i) if (tok[i] == "-d") { di = (int)i; break; }
        if (di != -1 && di + 3 < (int)tok.size()) {
            float dx = std::stof(tok[di + 1]), dy = std::stof(tok[di + 2]), dz = std::stof(tok[di + 3]);
            float m = extractMassFromTokens(tok, 1.0f);
            Shape s; s.pos = glm::vec3(x, y, z); s.scale = glm::vec3(dx, dy, dz); s.mass = m;
            s.type = ShapeType::Rect;
            s.id = (int)scene.rectsRef().size();
            s.inertia = computeLocalInertiaTensor(s);
            s.invInertia = glm::inverse(s.inertia);
            rects.push_back(s);
        }
        return;
    }
}

void drawConsole(Scene& scene) {
    static char commandBuf[256] = "";
    ImGui::Begin("Console");
    if (ImGui::InputText("Comando", commandBuf, sizeof(commandBuf), ImGuiInputTextFlags_EnterReturnsTrue)) {
        processCommand(scene, std::string(commandBuf));
        commandBuf[0] = '\0';
        ImGui::SetKeyboardFocusHere(-1);
    }
    ImGui::Text("Ex: add rect 0 1 0 -d 2 0.5 1 -p 3.5\nadd sphere 3 1 0 -p 2.0\nadd pyramid -3 1 0 -p 1.2");
    ImGui::End();

    // add mira no centro da tela
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    ImVec2 center = ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f, ImGui::GetIO().DisplaySize.y * 0.5f);
    float len = 6.0f; float thick = 2.0f; ImU32 col = IM_COL32(255, 255, 255, 200);
    dl->AddLine(ImVec2(center.x - len, center.y), ImVec2(center.x + len, center.y), col, thick);
    dl->AddLine(ImVec2(center.x, center.y - len), ImVec2(center.x, center.y + len), col, thick);
}