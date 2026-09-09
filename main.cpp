#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <cmath>
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
        vec2 clipPos = vec2(worldPos.x / uHalfScreen.x, -worldPos.y / uHalfScreen.y);
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

    RigidBody* bouncy = world.addBody(Vec2(130.0f, 180.0f), 5.0f,
                                      std::make_unique<CircleShape>(30.0f), 0.85f);
    RigidBody* damped = world.addBody(Vec2(-130.0f, 120.0f), 3.0f,
                                      std::make_unique<CircleShape>(24.0f), 0.15f);

    const CircleShape* bouncyShape = static_cast<const CircleShape*>(bouncy->shape.get());
    const CircleShape* dampedShape = static_cast<const CircleShape*>(damped->shape.get());

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

        glUniform4f(transformLoc, bouncy->position.x, bouncy->position.y,
                    bouncyShape->radius, bouncyShape->radius);
        glUniform4f(colorLoc, 0.35f, 0.7f, 0.95f, 1.0f);
        circleMesh.draw();

        glUniform4f(transformLoc, damped->position.x, damped->position.y,
                    dampedShape->radius, dampedShape->radius);
        glUniform4f(colorLoc, 0.85f, 0.55f, 0.35f, 1.0f);
        circleMesh.draw();

        glfwSwapBuffers(app.get());
        glfwPollEvents();
        ++renderedFrames;
    }

    return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}