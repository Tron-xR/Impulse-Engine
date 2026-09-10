#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <cstdlib>

#include "physics/Vec2.h"
#include "physics/RigidBody.h"
#include "physics/World.h"
#include "scenario_library.h"

constexpr unsigned int SCR_WIDTH = 800;
constexpr unsigned int SCR_HEIGHT = 600;
constexpr float PHYSICS_DT = 1.0f / 120.0f;

class GLFWWindow {
public:
    GLFWWindow(int width, int height, const char* title, bool visible) {
        if (!glfwInit()) {
            std::cerr << "GLFW init failed\n";
            std::exit(1);
        }
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win) {
            if (!visible) {
                std::cerr << "No display/GL context available — skipping headless "
                             "render smoke test (exit code 2). Physics is still "
                             "verified by the headless test suite.\n";
                glfwTerminate();
                std::exit(2);
            }
            std::cerr << "Window creation failed\n";
            glfwTerminate();
            std::exit(1);
        }
        glfwMakeContextCurrent(win);
        glfwSwapInterval(visible ? 1 : 0);
    }
    ~GLFWWindow() { if (win) glfwDestroyWindow(win); glfwTerminate(); }
    GLFWwindow* get() const { return win; }
private:
    GLFWWindow(const GLFWWindow&) = delete;
    GLFWWindow& operator=(const GLFWWindow&) = delete;
    GLFWwindow* win = nullptr;
};

struct Shader {
    GLuint id = 0;
    void compile(const char* vsrc, const char* fsrc) {
        auto compileStage = [](GLenum type, const char* src) -> GLuint {
            GLuint s = glCreateShader(type);
            glShaderSource(s, 1, &src, nullptr);
            glCompileShader(s);
            GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
            if (!ok) {
                char log[512]; glGetShaderInfoLog(s, 512, nullptr, log);
                std::cerr << "Shader compile error: " << log << "\n";
            }
            return s;
        };
        GLuint vs = compileStage(GL_VERTEX_SHADER, vsrc);
        GLuint fs = compileStage(GL_FRAGMENT_SHADER, fsrc);
        id = glCreateProgram();
        glAttachShader(id, vs); glAttachShader(id, fs);
        glLinkProgram(id);
        GLint ok; glGetProgramiv(id, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[512]; glGetProgramInfoLog(id, 512, nullptr, log);
            std::cerr << "Shader link error: " << log << "\n";
        }
        glDeleteShader(vs); glDeleteShader(fs);
    }
    void use() const { glUseProgram(id); }
    GLint loc(const char* name) const { return glGetUniformLocation(id, name); }
};

struct CircleMesh {
    GLuint vao = 0, vbo = 0;
    void init() {
        std::vector<float> verts;
        verts.push_back(0.0f); verts.push_back(0.0f);
        constexpr int N = 32;
        for (int i = 0; i <= N; ++i) {
            float a = 6.2831853f * i / N;
            verts.push_back(std::cos(a));
            verts.push_back(std::sin(a));
        }
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glBindVertexArray(0);
    }
    void draw() const { glBindVertexArray(vao); glDrawArrays(GL_TRIANGLE_FAN, 0, 34); glBindVertexArray(0); }
};

struct RectMesh {
    GLuint vao = 0, vbo = 0;
    void init() {
        float verts[] = { -0.5f,-0.5f, 0.5f,-0.5f, 0.5f,0.5f, -0.5f,-0.5f, 0.5f,0.5f, -0.5f,0.5f };
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glBindVertexArray(0);
    }
    void draw() const { glBindVertexArray(vao); glDrawArrays(GL_TRIANGLES, 0, 6); glBindVertexArray(0); }
};

struct LineMesh {
    GLuint vao = 0, vbo = 0;
    void init() {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glBindVertexArray(0);
    }
    void drawLine(const Vec2& a, const Vec2& b) const {
        float data[4] = { a.x, a.y, b.x, b.y };
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(data), data);
        glDrawArrays(GL_LINES, 0, 2);
        glBindVertexArray(0);
    }
};

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

int main(int argc, char** argv) {
    bool headless = false;
    int maxFrames = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--frames" && i + 1 < argc) {
            maxFrames = std::atoi(argv[i + 1]);
            headless = maxFrames > 0;
            ++i;
        }
    }

    GLFWWindow app(SCR_WIDTH, SCR_HEIGHT, "Impulse Engine", !headless);
    GLFWwindow* window = app.get();
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "GLAD init failed\n";
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    Shader shader;
    shader.compile(
        "#version 330 core\n"
        "layout(location=0) in vec2 aPos;\n"
        "uniform vec4 uTransform;\n"
        "uniform vec2 uHalfScreen;\n"
        "void main() {\n"
        "  vec2 worldPos = aPos * uTransform.zw + uTransform.xy;\n"
        "  vec2 ndc = vec2(worldPos.x / uHalfScreen.x, worldPos.y / uHalfScreen.y);\n"
        "  gl_Position = vec4(ndc, 0.0, 1.0);\n"
        "}\n",
        "#version 330 core\n"
        "uniform vec4 uColor;\n"
        "out vec4 FragColor;\n"
        "void main() { FragColor = uColor; }\n"
    );

    GLint hSL = shader.loc("uHalfScreen");
    GLint tL = shader.loc("uTransform");
    GLint cL = shader.loc("uColor");

    CircleMesh circleMesh;
    circleMesh.init();
    RectMesh rectMesh;
    rectMesh.init();
    LineMesh lineMesh;
    lineMesh.init();

    World world(Vec2(0.0f, -980.0f), PHYSICS_DT);

    int currentScenario = 2;
    loadScenario(world, currentScenario);

    bool paused = false;
    bool showOverlay = true;
    bool showDiagnostics = true;
    RigidBody* selectedBody = nullptr;
    float globalRestitution = 0.3f;
    float globalFriction = 0.5f;
    int spawnAsBox = 0;

    glfwSetWindowUserPointer(window, &world);

    double lastTime = glfwGetTime();
    int frameCount = 0;
    float fpsDisplay = 0.0f;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (headless && maxFrames > 0 && frameCount >= maxFrames) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiIO& ioFrame = ImGui::GetIO();

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);

        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            static bool spaceWasDown = false;
            if (!spaceWasDown) {
                paused = !paused;
                spaceWasDown = true;
            }
        } else {
            static bool spaceWasDown = false;
            spaceWasDown = false;
        }

        if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS && paused) {
            world.step();
        }

        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
            static bool rWasDown = false;
            if (!rWasDown) {
                loadScenario(world, currentScenario);
                selectedBody = nullptr;
                rWasDown = true;
            }
        } else {
            static bool rWasDown = false;
            rWasDown = false;
        }

        if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS) {
            static bool f1WasDown = false;
            if (!f1WasDown) { showOverlay = !showOverlay; f1WasDown = true; }
        } else {
            static bool f1WasDown = false;
            f1WasDown = false;
        }

        if (glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS) {
            static bool f2WasDown = false;
            if (!f2WasDown) { showDiagnostics = !showDiagnostics; f2WasDown = false; }
        } else {
            static bool f2WasDown = false;
            f2WasDown = false;
        }

        if (glfwGetKey(window, GLFW_KEY_DELETE) == GLFW_PRESS && selectedBody) {
            static bool delWasDown = false;
            if (!delWasDown) {
                world.removeBody(selectedBody);
                selectedBody = nullptr;
                delWasDown = true;
            }
        } else {
            static bool delWasDown = false;
            delWasDown = false;
        }

        static int mouseBtnState = 0;
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !ioFrame.WantCaptureMouse) {
            if (mouseBtnState == 0) {
                double mx, my;
                glfwGetCursorPos(window, &mx, &my);
                int wW, wH;
                glfwGetWindowSize(window, &wW, &wH);
                float worldX = (float)(2.0 * mx / wW - 1.0) * (SCR_WIDTH / 2.0f);
                float worldY = -(float)(2.0 * my / wH - 1.0) * (SCR_HEIGHT / 2.0f);
                Vec2 clickPos(worldX, worldY);

                RigidBody* hit = nullptr;
                float bestDist = 30.0f;
                for (auto& body : world.bodies) {
                    if (body->invMass == 0.0f) continue;
                    float dist = (body->position - clickPos).length();
                    if (body->shape->getType() == ShapeType::Circle) {
                        const CircleShape& cs = static_cast<const CircleShape&>(*body->shape);
                        if (dist <= cs.radius + 10.0f && dist < bestDist) {
                            bestDist = dist;
                            hit = body.get();
                        }
                    } else if (body->shape->getType() == ShapeType::Polygon) {
                        if (dist < bestDist + 20.0f) {
                            bestDist = dist;
                            hit = body.get();
                        }
                    }
                }

                if (hit) {
                    selectedBody = hit;
                } else {
                    if (spawnAsBox) {
                        world.addBody(clickPos, 1.0f,
                                      std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f, globalRestitution, globalFriction)),
                                      globalRestitution);
                    } else {
                        world.addBody(clickPos, 1.0f,
                                      std::make_unique<CircleShape>(15.0f), globalRestitution);
                    }
                }
                mouseBtnState = 1;
            }
        } else {
            mouseBtnState = 0;
        }

        if (!paused) {
            world.step();
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.12f, 0.12f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        shader.use();
        glUniform2f(hSL, SCR_WIDTH / 2.0f, SCR_HEIGHT / 2.0f);

        struct Rgb { float r, g, b; };
        const Rgb palette[] = {
            { 0.35f, 0.70f, 0.95f },
            { 0.85f, 0.55f, 0.35f },
            { 0.55f, 0.85f, 0.45f },
            { 0.90f, 0.80f, 0.30f },
            { 0.75f, 0.45f, 0.90f },
        };

        int colorIdx = 0;
        for (auto& body : world.bodies) {
            if (body->shape->getType() == ShapeType::Plane) {
                const PlaneShape& ps = static_cast<const PlaneShape&>(*body->shape);
                if (std::fabs(ps.normal.x) > 0.5f) {
                    glUniform4f(tL, body->position.x, body->position.y, 20.0f, 800.0f);
                } else {
                    glUniform4f(tL, body->position.x, body->position.y, 1600.0f, 20.0f);
                }
                glUniform4f(cL, 0.25f, 0.27f, 0.35f, 1.0f);
                rectMesh.draw();
            } else if (body->shape->getType() == ShapeType::Circle) {
                const CircleShape& cs = static_cast<const CircleShape&>(*body->shape);
                const Rgb& c = palette[colorIdx % 5];
                bool isSelected = (body.get() == selectedBody);
                glUniform4f(tL, body->position.x, body->position.y, cs.radius, cs.radius);
                glUniform4f(cL, c.r, c.g, c.b, isSelected ? 0.6f : 1.0f);
                circleMesh.draw();
                if (isSelected) {
                    glUniform4f(cL, 1.0f, 1.0f, 1.0f, 0.5f);
                    glUniform4f(tL, body->position.x, body->position.y, cs.radius + 3.0f, cs.radius + 3.0f);
                    circleMesh.draw();
                }
                ++colorIdx;
            } else if (body->shape->getType() == ShapeType::Polygon) {
                const PolygonShape& ps = static_cast<const PolygonShape&>(*body->shape);
                float hw = 0.0f, hh = 0.0f;
                for (const Vec2& v : ps.vertices) {
                    hw = std::max(hw, std::fabs(v.x));
                    hh = std::max(hh, std::fabs(v.y));
                }
                const Rgb& c = palette[colorIdx % 5];
                bool isSelected = (body.get() == selectedBody);
                glUniform4f(tL, body->position.x, body->position.y, hw, hh);
                glUniform4f(cL, c.r, c.g, c.b, isSelected ? 0.6f : 1.0f);
                rectMesh.draw();
                if (isSelected) {
                    glUniform4f(cL, 1.0f, 1.0f, 1.0f, 0.5f);
                    glUniform4f(tL, body->position.x, body->position.y, hw + 3.0f, hh + 3.0f);
                    rectMesh.draw();
                }
                ++colorIdx;
            }
        }

        if (showOverlay) {
            shader.use();
            for (auto& body : world.bodies) {
                if (body->invMass == 0.0f) continue;
                Vec2 end = body->position + body->velocity * 0.03f;
                glUniform4f(cL, 0.2f, 1.0f, 0.3f, 0.7f);
                lineMesh.drawLine(body->position, end);
            }
            for (auto& body : world.bodies) {
                if (body->shape->getType() == ShapeType::Circle) {
                    const CircleShape& cs = static_cast<const CircleShape&>(*body->shape);
                    glUniform4f(cL, 1.0f, 1.0f, 0.0f, 0.25f);
                    for (int i = 0; i < 16; ++i) {
                        float a0 = 6.2831853f * i / 16;
                        float a1 = 6.2831853f * (i + 1) / 16;
                        Vec2 p0 = body->position + Vec2(std::cos(a0), std::sin(a0)) * cs.radius;
                        Vec2 p1 = body->position + Vec2(std::cos(a1), std::sin(a1)) * cs.radius;
                        lineMesh.drawLine(p0, p1);
                    }
                } else if (body->shape->getType() == ShapeType::Polygon) {
                    const PolygonShape& ps = static_cast<const PolygonShape&>(*body->shape);
                    glUniform4f(cL, 1.0f, 1.0f, 0.0f, 0.25f);
                    for (size_t i = 0; i < ps.vertices.size(); ++i) {
                        Vec2 v0 = ps.vertices[i].rotated(body->angle) + body->position;
                        Vec2 v1 = ps.vertices[(i + 1) % ps.vertices.size()].rotated(body->angle) + body->position;
                        lineMesh.drawLine(v0, v1);
                    }
                }
            }
        }

        if (showDiagnostics) {
            frameCount++;
            double now = glfwGetTime();
            if (now - lastTime >= 0.5) {
                fpsDisplay = (float)frameCount / (float)(now - lastTime);
                frameCount = 0;
                lastTime = now;
            }

            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(320, 500), ImGuiCond_FirstUseEver);
            ImGui::Begin("Impulse Engine", nullptr, ImGuiWindowFlags_None);

            ImGui::Text("FPS: %.1f", fpsDisplay);
            ImGui::Text("Bodies: %d", (int)world.bodies.size());
            ImGui::Separator();

            ImGui::Text("Scenario");
            const auto& scenarios = getScenarioList();
            const char* currentName = (currentScenario >= 0 && currentScenario < (int)scenarios.size())
                ? scenarios[currentScenario].name : "Unknown";
            if (ImGui::BeginCombo("##scenario", currentName)) {
                for (int i = 0; i < (int)scenarios.size(); ++i) {
                    bool isSelected = (currentScenario == i);
                    if (ImGui::Selectable(scenarios[i].name, isSelected)) {
                        currentScenario = i;
                        loadScenario(world, currentScenario);
                        selectedBody = nullptr;
                    }
                    if (isSelected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            ImGui::Separator();
            ImGui::Text("Controls");
            if (ImGui::Button(paused ? "Resume (Space)" : "Pause (Space)")) paused = !paused;
            ImGui::SameLine();
            if (ImGui::Button("Step (.)") && paused) world.step();
            ImGui::SameLine();
            if (ImGui::Button("Reset (R)")) { loadScenario(world, currentScenario); selectedBody = nullptr; }

            ImGui::Separator();
            ImGui::Text("Spawn");
            ImGui::RadioButton("Circle", &spawnAsBox, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Box", &spawnAsBox, 1);
            if (ImGui::Button("Clear All")) { world.clearBodies(); selectedBody = nullptr; }

            ImGui::Separator();
            ImGui::Text("World");
            float grav[2] = { world.gravity.x, world.gravity.y };
            if (ImGui::DragFloat2("Gravity", grav, 5.0f, -2000.0f, 2000.0f)) {
                world.gravity = Vec2(grav[0], grav[1]);
            }

            ImGui::Separator();
            ImGui::Text("Global Defaults");
            ImGui::SliderFloat("Restitution##global", &globalRestitution, 0.0f, 1.0f);
            ImGui::SliderFloat("Friction##global", &globalFriction, 0.0f, 1.0f);

            if (selectedBody) {
                ImGui::Separator();
                ImGui::Text("Selected Body");
                ImGui::Text("Position: (%.1f, %.1f)", selectedBody->position.x, selectedBody->position.y);
                ImGui::Text("Velocity: (%.1f, %.1f)", selectedBody->velocity.x, selectedBody->velocity.y);

                float selRest = selectedBody->restitution;
                if (ImGui::SliderFloat("Restitution##sel", &selRest, 0.0f, 1.0f)) {
                    selectedBody->restitution = selRest;
                }

                if (selectedBody->shape) {
                    float selFric = selectedBody->shape->friction;
                    if (ImGui::SliderFloat("Friction##sel", &selFric, 0.0f, 1.0f)) {
                        selectedBody->shape->friction = selFric;
                    }
                }

                if (ImGui::Button("Delete Selected")) {
                    world.removeBody(selectedBody);
                    selectedBody = nullptr;
                }
            }

            ImGui::Separator();
            ImGui::Text("Diagnostics");

            if (world.profiler.getPhases().size() > 0) {
                ImGui::Text("Step time: %.1f us", world.profiler.getTotalMicroseconds());
                for (const auto& phase : world.profiler.getPhases()) {
                    float pct = (float)(phase.microseconds / world.profiler.getTotalMicroseconds() * 100.0);
                    ImGui::Text("  %s: %.1f us (%.0f%%)", phase.name, phase.microseconds, pct);
                }
            }

            if (world.detector.hasEvents()) {
                ImGui::Separator();
                ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "INSTABILITY DETECTED");
                for (const auto& evt : world.detector.getEvents()) {
                    ImGui::TextWrapped("%s", world.detector.describeEvent(evt).c_str());
                }
            }

            ImGui::Separator();
            ImGui::Checkbox("Overlay (F1)", &showOverlay);
            ImGui::Checkbox("Diagnostics (F2)", &showDiagnostics);
            ImGui::SameLine();
            ImGui::Text("[Space] Pause | [.] Step | [R] Reset");

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
