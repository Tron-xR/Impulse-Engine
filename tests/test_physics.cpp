#include <cmath>
#include <iostream>
#include <cstdlib>
#include <algorithm>

#include "physics/Vec2.h"
#include "physics/RigidBody.h"
#include "physics/World.h"

constexpr float TOLERANCE = 1e-3f;

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " #cond " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

bool approxEqual(float a, float b, float tol = TOLERANCE) {
    return std::fabs(a - b) < tol;
}

bool approxEqualRel(float a, float b, float relTol) {
    return std::fabs(a - b) < relTol * std::max({ 1.0f, std::fabs(a), std::fabs(b) });
}

bool approxEqual(const Vec2& a, const Vec2& b, float tol = TOLERANCE) {
    return approxEqual(a.x, b.x, tol) && approxEqual(a.y, b.y, tol);
}

void testVec2Math() {
    Vec2 a(3.0f, 4.0f);
    Vec2 b(1.0f, 2.0f);

    CHECK(a + b == Vec2(4.0f, 6.0f));
    CHECK(a - b == Vec2(2.0f, 2.0f));
    CHECK(a * 2.0f == Vec2(6.0f, 8.0f));
    CHECK(2.0f * a == Vec2(6.0f, 8.0f));
    CHECK(-a == Vec2(-3.0f, -4.0f));

    CHECK(approxEqual(a.length(), 5.0f));
    CHECK(approxEqual(a.dot(b), 11.0f));
    CHECK(approxEqual(a.cross(b), 2.0f));

    Vec2 n = a.normalized();
    CHECK(approxEqual(n.length(), 1.0f));

    std::cout << "  PASS: Vec2 math\n";
}

void testSemiImplicitEulerFreeFall() {
    const float mass = 2.0f;
    const float dt = 1.0f / 120.0f;
    const float gravity = -980.0f;
    const float duration = 1.0f;
    const int steps = static_cast<int>(duration / dt);

    World world(Vec2(0.0f, gravity), dt);
    RigidBody* body = world.addBody(Vec2(0.0f, 100.0f), mass, std::make_unique<CircleShape>(10.0f));

    for (int i = 0; i < steps; ++i) {
        world.step();
    }

    float t = steps * dt;
    float analyticalVelocity = gravity * t;
    CHECK(approxEqualRel(body->velocity.y, analyticalVelocity, 1e-4f));

    float errorBound = 0.5f * std::fabs(gravity) * dt * t;
    float analyticalPosition = 100.0f + 0.5f * gravity * t * t;
    CHECK(approxEqual(body->position.y, analyticalPosition, errorBound + 0.01f));

    std::cout << "  PASS: Semi-implicit Euler free-fall (~1s, " << steps << " steps)\n";
    std::cout << "    Simulated pos.y=" << body->position.y
              << " vel.y=" << body->velocity.y << "\n";
    std::cout << "    Analytical  pos.y=" << analyticalPosition
              << " vel.y=" << analyticalVelocity
              << " (pos within " << errorBound + 0.01f << " px)\n";
}

void testStepCountTruncationRegression() {
    const float dt = 1.0f / 120.0f;
    const float duration = 1.0f;
    const int steps = static_cast<int>(duration / dt);

    CHECK(steps == 119);

    const float gravity = -980.0f;
    World world(Vec2(0.0f, gravity), dt);
    RigidBody* body = world.addBody(Vec2(0.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(5.0f));

    for (int i = 0; i < steps; ++i) {
        world.step();
    }

    float expectedVelocity = gravity * steps * dt;
    CHECK(approxEqualRel(body->velocity.y, expectedVelocity, 1e-4f));
    CHECK(std::fabs(body->velocity.y) < std::fabs(gravity * duration));

    std::cout << "  PASS: Step-count truncation regression (1.0f/120.0f -> 119 steps, not 120)\n";
    std::cout << "    duration/dt=" << duration / dt << " steps=" << steps
              << " vel.y=" << body->velocity.y
              << " expected=" << expectedVelocity << "\n";
}

void testSemiImplicitEulerShortDuration() {
    const float mass = 1.0f;
    const float dt = 1.0f / 60.0f;
    const float gravity = -980.0f;
    const int steps = 10;

    World world(Vec2(0.0f, gravity), dt);
    RigidBody* body = world.addBody(Vec2(0.0f, 500.0f), mass, std::make_unique<CircleShape>(5.0f));

    for (int i = 0; i < steps; ++i) {
        world.step();
    }

    float t = steps * dt;
    float analyticalVelocity = gravity * t;
    CHECK(approxEqualRel(body->velocity.y, analyticalVelocity, 1e-4f));

    float errorBound = 0.5f * std::fabs(gravity) * dt * t;
    float analyticalPosition = 500.0f + 0.5f * gravity * t * t;
    CHECK(approxEqual(body->position.y, analyticalPosition, errorBound + 0.01f));

    std::cout << "  PASS: Semi-implicit Euler free-fall (short, " << steps << " steps)\n";
    std::cout << "    Simulated pos.y=" << body->position.y
              << " vel.y=" << body->velocity.y << "\n";
    std::cout << "    Analytical  pos.y=" << analyticalPosition
              << " vel.y=" << analyticalVelocity
              << " (pos within " << errorBound + 0.01f << " px)\n";
}

void testStaticBodyDoesNotMove() {
    World world(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    RigidBody* body = world.addBody(Vec2(0.0f, 0.0f), 0.0f, std::make_unique<CircleShape>(10.0f));

    for (int i = 0; i < 120; ++i) {
        world.step();
    }

    CHECK(body->position == Vec2(0.0f, 0.0f));
    CHECK(body->velocity == Vec2(0.0f, 0.0f));

    std::cout << "  PASS: Static body (zero mass) does not move\n";
}

int main() {
    std::cout << "ImpulseEngine Unit Tests\n";
    std::cout << "========================\n";

    testVec2Math();
    testSemiImplicitEulerFreeFall();
    testSemiImplicitEulerShortDuration();
    testStepCountTruncationRegression();
    testStaticBodyDoesNotMove();

    std::cout << "========================\n";
    std::cout << "All tests passed.\n";
    return 0;
}
