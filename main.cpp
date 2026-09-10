#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <cstdlib>

#include "physics/Vec2.h"
#include "physics/RigidBody.h"
#include "physics/World.h"

constexpr unsigned int SCR_WIDTH = 800;
constexpr unsigned int SCR_HEIGHT = 600;
constexpr float PHYSICS_DT = 1.0f / 120.0f;

class GLFWWindow {
public:
    GLFWWindow(int width, int height, const char* title) {
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        window = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (window == nullptr) {
            std::cout << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
        }
    }

    ~GLFWWindow() {
        if (window) {
            glfwDestroyWindow(window);
            glfwTerminate();
        }
    }

    GLFWWindow(const GLFWWindow&) = delete;
    GLFWWindow& operator=(const GLFWWindow&) = delete;

    GLFWwindow* get() const { return window; }
    bool shouldClose() const { return glfwWindowShouldClose(window); }

private:
    GLFWwindow* window = nullptr;
};

class ShaderProgram {
public:
    ShaderProgram(const char* vertexSrc, const char* fragmentSrc) {
        unsigned int vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
        unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
        if (vs == 0 || fs == 0) return;

        program = glCreateProgram();
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);

        int success;
        char infoLog[512];
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(program, 512, nullptr, infoLog);
            std::cout << "Shader program linking failed:\n" << infoLog << std::endl;
        }

        glDeleteShader(vs);
        glDeleteShader(fs);
    }

    ~ShaderProgram() {
        if (program) glDeleteProgram(program);
    }

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;

    void use() const { glUseProgram(program); }
    unsigned int get() const { return program; }

private:
    static unsigned int compileShader(GLenum type, const char* src) {
        unsigned int shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);

        int success;
        char infoLog[512];
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 512, nullptr, infoLog);
            std::cout << "Shader compilation failed:\n" << infoLog << std::endl;
            return 0;
        }
        return shader;
    }

    unsigned int program = 0;
};

class Mesh {
public:
    Mesh(const std::vector<float>& verts, GLenum mode) : mode(mode) {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                     verts.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);

        vertexCount = static_cast<GLsizei>(verts.size() / 2);
    }

    ~Mesh() {
        glDeleteBuffers(1, &vbo);
        glDeleteVertexArrays(1, &vao);
    }

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    void draw() const {
        glBindVertexArray(vao);
        glDrawArrays(mode, 0, vertexCount);
    }

private:
    unsigned int vao = 0;
    unsigned int vbo = 0;
    GLsizei vertexCount = 0;
    GLenum mode;
};

Mesh createCircleMesh(int segments) {
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>(segments + 1) * 2);
    verts.push_back(0.0f);
    verts.push_back(0.0f);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * PI * static_cast<float>(i) / static_cast<float>(segments);
        verts.push_back(std::cos(angle));
        verts.push_back(std::sin(angle));
    }
    return Mesh(verts, GL_TRIANGLE_FAN);
}

Mesh createRectMesh() {
    std::vector<float> verts = {
        -0.5f, -0.5f,
         0.5f, -0.5f,
        -0.5f,  0.5f,
         0.5f,  0.5f
    };
    return Mesh(verts, GL_TRIANGLE_STRIP);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

int main(int argc, char* argv[]) {
    long autoCloseFrames = 0;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--frames" && i + 1 < argc) {
            autoCloseFrames = std::atol(argv[++i]);
        }
    }

    GLFWWindow app(SCR_WIDTH, SCR_HEIGHT, "ImpulseEngine");
    if (app.get() == nullptr) return -1;

    glfwMakeContextCurrent(app.get());
    if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) == 0) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
    glfwSetFramebufferSizeCallback(app.get(), framebuffer_size_callback);

    const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    uniform vec4 uTransform;    // x, y, scaleX, scaleY
    uniform vec2 uHalfScreen;
    void main() {
        vec2 worldPos = aPos * uTransform.zw + uTransform.xy;
        vec2 clipPos = vec2(worldPos.x / uHalfScreen.x, worldPos.y / uHalfScreen.y);
        gl_Position = vec4(clipPos, 0.0, 1.0);
    }
    )";

    const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    uniform vec4 uColor;
    void main() {
        FragColor = uColor;
    }
    )";

    ShaderProgram shader(vertexShaderSource, fragmentShaderSource);
    if (shader.get() == 0) return -1;

    Mesh circleMesh = createCircleMesh(24);
    Mesh rectMesh = createRectMesh();
    GLint transformLoc = glGetUniformLocation(shader.get(), "uTransform");
    GLint halfScreenLoc = glGetUniformLocation(shader.get(), "uHalfScreen");
    GLint colorLoc = glGetUniformLocation(shader.get(), "uColor");

    World world(Vec2(0.0f, -980.0f), PHYSICS_DT);

    constexpr float FLOOR_Y = -220.0f;
    auto floorPlane = std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), FLOOR_Y, 0.0f);
    world.addBody(Vec2(0.0f, FLOOR_Y - 10.0f), 0.0f, std::move(floorPlane));

    constexpr float BOX_HALF = 25.0f;
    for (int i = 0; i < 10; ++i) {
        RigidBody* box = world.addBody(Vec2(-300.0f, FLOOR_Y + BOX_HALF + 2.0f * BOX_HALF * i), 1.0f,
                                       std::make_unique<PolygonShape>(PolygonShape::makeBox(BOX_HALF, BOX_HALF, 0.0f)), 0.0f);
        box->position.y = FLOOR_Y + BOX_HALF + 2.0f * BOX_HALF * i;
    }

    struct Spawn { Vec2 pos; float mass; float radius; Vec2 vel; float restitution; };
    const Spawn spawns[] = {
        { { -110.0f, 150.0f }, 2.0f, 22.0f, {  60.0f,   0.0f }, 0.80f },
        { {   10.0f, 260.0f }, 3.0f, 18.0f, { -40.0f,   0.0f }, 0.65f },
        { {  130.0f, 200.0f }, 4.0f, 26.0f, {  30.0f,   0.0f }, 0.90f },
        { {  250.0f, 320.0f }, 2.0f, 20.0f, { -50.0f,   0.0f }, 0.50f },
        { {  340.0f,  60.0f }, 3.0f, 16.0f, { -20.0f,   0.0f }, 0.75f },
    };

    for (const Spawn& s : spawns) {
        RigidBody* body = world.addBody(s.pos, s.mass,
                                        std::make_unique<CircleShape>(s.radius), s.restitution);
        body->velocity = s.vel;
    }

    RigidBody* fallingBox = world.addBody(Vec2(200.0f, 350.0f), 2.0f,
                                          std::make_unique<PolygonShape>(PolygonShape::makeBox(30.0f, 30.0f, 0.0f)), 0.5f);

    glClearColor(0.15f, 0.15f, 0.2f, 1.0f);

    double lastTime = glfwGetTime();
    double accumulator = 0.0;
    long renderedFrames = 0;

    while (!app.shouldClose()) {
        if (autoCloseFrames > 0 && renderedFrames >= autoCloseFrames) {
            glfwSetWindowShouldClose(app.get(), true);
        }

        if (glfwGetKey(app.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(app.get(), true);
        if (glfwGetKey(app.get(), GLFW_KEY_SPACE) == GLFW_PRESS)
            glfwSetWindowShouldClose(app.get(), true);

        double now = glfwGetTime();
        double frameTime = now - lastTime;
        lastTime = now;
        accumulator += frameTime;

        int maxSteps = 8;
        while (accumulator >= PHYSICS_DT) {
            world.step();
            accumulator -= PHYSICS_DT;
            if (--maxSteps <= 0) {
                accumulator = 0.0;
                break;
            }
        }

        glClear(GL_COLOR_BUFFER_BIT);

        shader.use();
        glUniform2f(halfScreenLoc, SCR_WIDTH / 2.0f, SCR_HEIGHT / 2.0f);

        glUniform4f(transformLoc, 0.0f, FLOOR_Y - 10.0f, 1600.0f, 20.0f);
        glUniform4f(colorLoc, 0.32f, 0.34f, 0.42f, 1.0f);
        rectMesh.draw();

        struct Rgb { float r, g, b; };
        const Rgb palette[] = {
            { 0.35f, 0.70f, 0.95f },
            { 0.85f, 0.55f, 0.35f },
            { 0.55f, 0.85f, 0.45f },
            { 0.90f, 0.80f, 0.30f },
            { 0.75f, 0.45f, 0.90f },
        };

        int spawned = 0;
        for (auto& body : world.bodies) {
            if (body->shape->getType() != ShapeType::Circle) continue;

            const CircleShape& cs = static_cast<const CircleShape&>(*body->shape);
            const Rgb& c = palette[spawned % 5];
            glUniform4f(transformLoc, body->position.x, body->position.y, cs.radius, cs.radius);
            glUniform4f(colorLoc, c.r, c.g, c.b, 1.0f);
            circleMesh.draw();
            ++spawned;
        }

        int boxes = 0;
        for (auto& body : world.bodies) {
            if (body->shape->getType() != ShapeType::Polygon) continue;

            const PolygonShape& ps = static_cast<const PolygonShape&>(*body->shape);
            float hw = 0.0f, hh = 0.0f;
            for (const Vec2& v : ps.vertices) {
                hw = std::max(hw, std::fabs(v.x));
                hh = std::max(hh, std::fabs(v.y));
            }
            const Rgb& c = palette[(spawned + boxes) % 5];
            glUniform4f(transformLoc, body->position.x, body->position.y, hw, hh);
            glUniform4f(colorLoc, c.r, c.g, c.b, 1.0f);
            rectMesh.draw();
            ++boxes;
        }

        glfwSwapBuffers(app.get());
        glfwPollEvents();
        ++renderedFrames;
    }

    return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}