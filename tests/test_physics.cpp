#include <cmath>
#include <iostream>
#include <cstdlib>
#include <algorithm>

#include "physics/Vec2.h"
#include "physics/RigidBody.h"
#include "physics/World.h"
#include "collision/CollisionDetect.h"
#include "diagnostics/Diagnostics.h"
#include "scenario_library.h"

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

void testPlaneCollisionDetection() {
    PlaneShape floor(Vec2(0.0f, 1.0f), -220.0f);
    Manifold m;

    RigidBody touching(Vec2(0.0f, -190.0f), 1.0f, std::make_unique<CircleShape>(30.0f));
    CHECK(!detectCircleVsPlane(touching, floor, m));

    RigidBody overlapping(Vec2(0.0f, -200.0f), 1.0f, std::make_unique<CircleShape>(30.0f));
    CHECK(detectCircleVsPlane(overlapping, floor, m));
    CHECK(m.normal == Vec2(0.0f, 1.0f));
    CHECK(approxEqual(m.penetration, 10.0f));
    CHECK(approxEqual(m.contactPoint, Vec2(0.0f, -220.0f)));
    CHECK(approxEqual(m.restitution, 0.3f));

    std::cout << "  PASS: Circle-vs-plane detection (normal/penetration/contact)\n";
}

void testCirclePlaneImpulseKnownAnswer() {
    PlaneShape floor(Vec2(0.0f, 1.0f), -220.0f);
    RigidBody staticPlane(Vec2(0.0f, -230.0f), 0.0f, std::make_unique<PlaneShape>(floor));
    Manifold m;

    RigidBody body(Vec2(0.0f, -200.0f), 2.0f, std::make_unique<CircleShape>(30.0f), 0.5f);
    body.velocity = Vec2(0.0f, -40.0f);

    CHECK(detectCircleVsPlane(body, floor, m));
    CHECK(approxEqual(m.restitution, 0.5f));
    applyImpulse(body, staticPlane, m);
    CHECK(approxEqual(body.velocity.y, 20.0f));
    CHECK(approxEqual(body.velocity.x, 0.0f));

    RigidBody heavy(Vec2(0.0f, -200.0f), 10.0f, std::make_unique<CircleShape>(30.0f), 0.5f);
    heavy.velocity = Vec2(0.0f, -40.0f);
    CHECK(detectCircleVsPlane(heavy, floor, m));
    applyImpulse(heavy, staticPlane, m);
    CHECK(approxEqual(heavy.velocity.y, 20.0f));

    std::cout << "  PASS: Circle-vs-plane known-answer impulse (v_out = e * v_in, mass-independent)\n";
}

void testDropBouncesAndDoesNotSink() {
    World world(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    constexpr float FLOOR_Y = -220.0f;
    constexpr float RADIUS = 30.0f;
    world.addBody(Vec2(0.0f, FLOOR_Y - 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), FLOOR_Y));

    RigidBody* c = world.addBody(Vec2(0.0f, 180.0f), 2.0f,
                                 std::make_unique<CircleShape>(RADIUS), 0.6f);

    const float restY = FLOOR_Y + RADIUS;
    bool bounced = false;

    for (int i = 0; i < 2400; ++i) {
        world.step();
        CHECK(std::isfinite(c->position.x));
        CHECK(std::isfinite(c->position.y));
        CHECK(std::isfinite(c->velocity.x));
        CHECK(std::isfinite(c->velocity.y));
        if (c->velocity.y > 100.0f) bounced = true;
        CHECK(c->position.y > restY - 12.0f);
    }

    std::cout << "  PASS: Dropped circle bounces and settles at rest height (no sink)\n";
    std::cout << "    final pos.y=" << c->position.y << " vel.y=" << c->velocity.y << " restY=" << restY << "\n";
    CHECK(bounced);
    CHECK(std::fabs(c->position.y - restY) < 10.0f);
    CHECK(std::fabs(c->velocity.y) < 10.0f);
}

void testCircleCircleDetection() {
    RigidBody a(Vec2(0.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    RigidBody b(Vec2(15.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    Manifold m;

    CHECK(detectCircleCircle(a, b, m));
    CHECK(approxEqual(m.normal, Vec2(-1.0f, 0.0f)));
    CHECK(approxEqual(m.penetration, 5.0f));
    CHECK(approxEqual(m.restitution, 0.3f));

    RigidBody c(Vec2(0.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    RigidBody d(Vec2(25.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    CHECK(!detectCircleCircle(c, d, m));

    std::cout << "  PASS: Circle-circle detection (normal/penetration, separated case)\n";
}

void testEqualMassElasticVelocitySwap() {
    World world(Vec2(0.0f, 0.0f), 1.0f / 120.0f);
    RigidBody* a = world.addBody(Vec2(-12.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f), 1.0f);
    RigidBody* b = world.addBody(Vec2(12.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f), 1.0f);
    a->velocity = Vec2(20.0f, 0.0f);
    b->velocity = Vec2(-20.0f, 0.0f);

    for (int i = 0; i < 60; ++i) {
        world.step();
    }

    CHECK(approxEqual(a->velocity.x, -20.0f, 0.01f));
    CHECK(approxEqual(b->velocity.x, 20.0f, 0.01f));
    CHECK(approxEqual(a->velocity.y, 0.0f, 0.01f));
    CHECK(approxEqual(b->velocity.y, 0.0f, 0.01f));
    CHECK(approxEqual(a->velocity.x + b->velocity.x, 0.0f, 0.02f));

    std::cout << "  PASS: Equal-mass elastic collision swaps velocities (done criterion)\n";
    std::cout << "    vA=(" << a->velocity.x << ", " << a->velocity.y
              << ") vB=(" << b->velocity.x << ", " << b->velocity.y << ")\n";
}

void testMomentumConservedUnequalMass() {
    World world(Vec2(0.0f, 0.0f), 1.0f / 120.0f);
    RigidBody* a = world.addBody(Vec2(-12.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f), 1.0f);
    RigidBody* b = world.addBody(Vec2(12.0f, 0.0f), 3.0f, std::make_unique<CircleShape>(10.0f), 1.0f);
    a->velocity = Vec2(20.0f, 0.0f);
    b->velocity = Vec2(-20.0f, 0.0f);

    float momentumBefore = a->velocity.x + 3.0f * b->velocity.x;

    for (int i = 0; i < 60; ++i) {
        world.step();
    }

    float momentumAfter = a->velocity.x + 3.0f * b->velocity.x;
    CHECK(approxEqual(momentumAfter, momentumBefore, 0.02f));
    CHECK(approxEqual(a->velocity.x, -40.0f, 0.02f));
    CHECK(approxEqual(b->velocity.x, 0.0f, 0.02f));

    std::cout << "  PASS: Unequal-mass elastic collision (momentum conserved, vA'=-40, vB'=0)\n";
}

void testNonElasticCollisionLosesRelativeSpeed() {
    RigidBody a(Vec2(0.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(5.0f), 0.5f);
    RigidBody b(Vec2(9.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(5.0f), 0.5f);
    a.velocity = Vec2(20.0f, 0.0f);
    b.velocity = Vec2(-20.0f, 0.0f);

    Manifold m;
    CHECK(detectCircleCircle(a, b, m));
    CHECK(approxEqual(m.restitution, 0.5f));
    applyImpulse(a, b, m);

    Vec2 rvBefore = Vec2(20.0f, 0.0f) - Vec2(-20.0f, 0.0f);
    Vec2 rvAfter = a.velocity - b.velocity;
    CHECK(approxEqual(a.velocity.x, -10.0f, 1e-4f));
    CHECK(approxEqual(b.velocity.x, 10.0f, 1e-4f));
    CHECK(approxEqual(rvAfter.length(), 0.5f * rvBefore.length(), 1e-4f));

    std::cout << "  PASS: e=0.5 collision halves relative speed (40 -> 20)\n";
}

void testIdenticalPositionCirclesNoNaN() {
    World world(Vec2(0.0f, 0.0f), 1.0f / 120.0f);
    RigidBody* a = world.addBody(Vec2(0.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(5.0f), 0.8f);
    RigidBody* b = world.addBody(Vec2(0.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(5.0f), 0.8f);
    a->velocity = Vec2(5.0f, 0.0f);
    b->velocity = Vec2(-5.0f, 0.0f);

    Manifold m;
    CHECK(detectCircleCircle(*a, *b, m));
    CHECK(std::isfinite(m.normal.x) && std::isfinite(m.normal.y));

    for (int i = 0; i < 30; ++i) {
        world.step();
    }

    CHECK(std::isfinite(a->position.x) && std::isfinite(a->position.y));
    CHECK(std::isfinite(b->position.x) && std::isfinite(b->position.y));
    CHECK(std::isfinite(a->velocity.x) && std::isfinite(a->velocity.y));
    CHECK(std::isfinite(b->velocity.x) && std::isfinite(b->velocity.y));
    CHECK(approxEqual(a->velocity.x + b->velocity.x, 0.0f, 0.01f));

    std::cout << "  PASS: Identically-positioned circles (epsilon guard, no NaN)\n";
}

void testPolygonVsPlaneDetectionAndImpulse() {
    PlaneShape floor(Vec2(0.0f, 1.0f), -220.0f);
    RigidBody staticPlane(Vec2(0.0f, -230.0f), 0.0f, std::make_unique<PlaneShape>(floor));

    RigidBody resting(Vec2(0.0f, -190.0f), 2.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(30.0f, 30.0f)), 0.5f);
    Manifold m;
    CHECK(!detectPolygonVsPlane(resting, floor, m));

    RigidBody falling(Vec2(0.0f, -200.0f), 2.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(30.0f, 30.0f)), 0.5f);
    falling.velocity = Vec2(0.0f, -40.0f);
    CHECK(detectPolygonVsPlane(falling, floor, m));
    CHECK(approxEqual(m.normal, Vec2(0.0f, 1.0f)));
    CHECK(approxEqual(m.penetration, 10.0f));

    applyImpulse(falling, staticPlane, m);
    CHECK(approxEqual(falling.velocity.y, 20.0f));
    CHECK(approxEqual(falling.velocity.x, 0.0f));

    RigidBody rotated(Vec2(0.0f, -220.0f - 2.0f + 14.142f), 2.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(10.0f, 10.0f)), 0.9f);
    rotated.angle = PI / 4.0f;
    rotated.velocity = Vec2(0.0f, -40.0f);
    CHECK(detectPolygonVsPlane(rotated, floor, m));
    CHECK(approxEqual(m.normal, Vec2(0.0f, 1.0f)));
    CHECK(approxEqualRel(m.penetration, 2.0f, 1e-3f));
    applyImpulse(rotated, staticPlane, m);
    CHECK(approxEqual(rotated.velocity.y, 0.9f * 40.0f, 1e-3f));

    std::cout << "  PASS: Polygon-vs-plane detection + impulse (box, rotated box, separated)\n";
}

void testPolygonPolygonDetection() {
    RigidBody a(Vec2(0.0f, 0.0f), 1.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f)));
    RigidBody b(Vec2(30.0f, 0.0f), 1.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f)));
    Manifold m;

    CHECK(detectPolygonPolygon(a, b, m));
    CHECK(approxEqual(m.normal, Vec2(-1.0f, 0.0f)));
    CHECK(approxEqual(m.penetration, 10.0f));

    RigidBody separated(Vec2(50.0f, 0.0f), 1.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f)));
    CHECK(!detectPolygonPolygon(a, separated, m));

    RigidBody touching(Vec2(40.0f, 0.0f), 1.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f)));
    Manifold t;
    CHECK(!detectPolygonPolygon(a, touching, t));

    std::cout << "  PASS: Polygon-polygon SAT (overlap axis, separated, touching)\n";
}

void testCirclePolygonDetection() {
    RigidBody box(Vec2(0.0f, 0.0f), 10.0f, std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f)));
    Manifold m;

    RigidBody onTop(Vec2(0.0f, 25.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    CHECK(detectCirclePolygon(onTop, box, m));
    CHECK(approxEqual(m.normal, Vec2(0.0f, 1.0f)));
    CHECK(approxEqual(m.penetration, 5.0f));

    RigidBody inside(Vec2(0.0f, 15.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    CHECK(detectCirclePolygon(inside, box, m));
    CHECK(approxEqual(m.normal, Vec2(0.0f, 1.0f)));
    CHECK(approxEqual(m.penetration, 5.0f));

    RigidBody beside(Vec2(25.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    CHECK(detectCirclePolygon(beside, box, m));
    CHECK(approxEqual(m.normal, Vec2(1.0f, 0.0f)));
    CHECK(approxEqual(m.penetration, 5.0f));

    RigidBody away(Vec2(100.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f));
    CHECK(!detectCirclePolygon(away, box, m));

    std::cout << "  PASS: Circle-polygon SAT (above, inside, beside, separated)\n";
}

void testBoxStackStable() {
    World world(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    constexpr float FLOOR_Y = -220.0f;
    constexpr float HALF = 30.0f;
    world.addBody(Vec2(0.0f, FLOOR_Y - 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), FLOOR_Y, 0.0f));

    constexpr int COUNT = 4;
    RigidBody* stack[COUNT];
    for (int i = 0; i < COUNT; ++i) {
        stack[i] = world.addBody(Vec2(0.0f, FLOOR_Y + HALF + 2.0f * HALF * i), 1.0f,
                                 std::make_unique<PolygonShape>(PolygonShape::makeBox(HALF, HALF)), 0.0f);
    }
    for (int i = 0; i < COUNT; ++i) {
        stack[i]->position.y = FLOOR_Y + HALF + 2.0f * HALF * i;
    }

    for (int i = 0; i < 2400; ++i) {
        world.step();
    }

    for (int i = 0; i < COUNT; ++i) {
        float expectedY = FLOOR_Y + HALF + 2.0f * HALF * i;
        std::cout << "    box" << i << " expectedY=" << expectedY
                  << " y=" << stack[i]->position.y << " vy=" << stack[i]->velocity.y
                  << " y-expected=" << (stack[i]->position.y - expectedY) << "\n";
        CHECK(std::isfinite(stack[i]->position.x) && std::isfinite(stack[i]->position.y));
        CHECK(std::fabs(stack[i]->position.y - expectedY) < 1.0f);
        CHECK(std::fabs(stack[i]->position.x) < 0.5f);
        CHECK(std::fabs(stack[i]->velocity.y) < 10.0f);
    }

    std::cout << "  PASS: 4-box stack stays stable (M4 done criterion)\n";
}

void testFrictionStopsSlidingBlock() {
    World world(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    constexpr float FLOOR_Y = -220.0f;
    world.addBody(Vec2(0.0f, FLOOR_Y - 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), FLOOR_Y, 0.0f, 1.0f));

    RigidBody* grip = world.addBody(Vec2(-100.0f, FLOOR_Y + 20.0f), 1.0f,
                                    std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f, 0.0f, 1.0f)), 0.0f);
    grip->velocity.x = 100.0f;

    RigidBody* ice = world.addBody(Vec2(100.0f, FLOOR_Y + 20.0f), 1.0f,
                                   std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f, 0.0f, 0.0f)), 0.0f);
    ice->velocity.x = 100.0f;

    for (int i = 0; i < 600; ++i) {
        world.step();
        if (!std::isfinite(grip->position.x) || !std::isfinite(ice->position.x)) break;
    }

    CHECK(std::fabs(grip->velocity.x) < 2.0f);
    CHECK(std::fabs(ice->velocity.x - 100.0f) < 2.0f);

    std::cout << "  PASS: friction stops a sliding block (vx " << grip->velocity.x
              << ") while u=0 block keeps sliding (vx " << ice->velocity.x << ")\n";
}

void testFrictionRampOrdering() {
    constexpr float FLOOR_Y = -220.0f;
    constexpr float INITIAL_VX = 150.0f;
    constexpr int STEPS = 400;
    float travelled[3] = { 0.0f, 0.0f, 0.0f };

    const float frictions[3] = { 0.0f, 0.3f, 1.0f };
    for (int k = 0; k < 3; ++k) {
        World world(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
        world.addBody(Vec2(0.0f, FLOOR_Y - 10.0f), 0.0f,
                      std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), FLOOR_Y, 0.0f, 1.0f));
        RigidBody* block = world.addBody(
            Vec2(0.0f, FLOOR_Y + 20.0f), 1.0f,
            std::make_unique<PolygonShape>(PolygonShape::makeBox(20.0f, 20.0f, 0.0f, frictions[k])), 0.0f);
        block->velocity.x = INITIAL_VX;

        for (int i = 0; i < STEPS; ++i) {
            world.step();
        }
        travelled[k] = block->position.x;
    }

    CHECK(std::fabs(travelled[0] - INITIAL_VX / 120.0f * STEPS) < 5.0f);
    CHECK(travelled[2] < travelled[1]);
    CHECK(travelled[1] < travelled[0] * 0.5f);

    std::cout << "  PASS: friction ramp ordering (u=1.0 travelled=" << travelled[2]
              << " < u=0.3 travelled=" << travelled[1]
              << " < u=0.0 travelled=" << travelled[0] << ")\n";
}

void testBoxStack10Stable() {
    World world(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    constexpr float FLOOR_Y = -220.0f;
    constexpr float HALF = 25.0f;
    constexpr int COUNT = 10;
    world.addBody(Vec2(0.0f, FLOOR_Y - 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), FLOOR_Y, 0.0f, 0.6f));

    std::vector<RigidBody*> stack;
    stack.reserve(COUNT);
    for (int i = 0; i < COUNT; ++i) {
        RigidBody* box = world.addBody(
            Vec2(0.0f, FLOOR_Y + HALF + 2.0f * HALF * i), 1.0f,
            std::make_unique<PolygonShape>(PolygonShape::makeBox(HALF, HALF, 0.0f, 0.5f)), 0.0f);
        stack.push_back(box);
    }

    for (int i = 0; i < 2400; ++i) {
        world.step();
    }

    float maxSink = 0.0f;
    for (int i = 0; i < COUNT; ++i) {
        float expectedY = FLOOR_Y + HALF + 2.0f * HALF * i;
        std::cout << "    box" << i << " expectedY=" << expectedY
                  << " y=" << stack[i]->position.y << " vy=" << stack[i]->velocity.y
                  << " y-expected=" << (stack[i]->position.y - expectedY) << "\n";
        maxSink = std::max(maxSink, std::fabs(stack[i]->position.y - expectedY));
        CHECK(std::fabs(stack[i]->position.y - expectedY) < 3.0f);
        CHECK(std::fabs(stack[i]->position.x) < 0.5f);
        CHECK(std::fabs(stack[i]->velocity.y) < 25.0f);
    }

    std::cout << "  PASS: 10-box stack stays stable (M5 done criterion, max sink " << maxSink << "px)\n";
}

void testRestingNoBounce() {
    World world(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    constexpr float FLOOR_Y = -220.0f;
    constexpr float RADIUS = 20.0f;
    world.addBody(Vec2(0.0f, FLOOR_Y - 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), FLOOR_Y, 0.0f));

    RigidBody* c = world.addBody(Vec2(0.0f, 0.0f), 1.0f,
                                 std::make_unique<CircleShape>(RADIUS), 0.0f);

    for (int i = 0; i < 2400; ++i) {
        world.step();
    }

    const float restY = FLOOR_Y + RADIUS;
    CHECK(std::fabs(c->position.y - restY) < 8.0f);
    CHECK(std::fabs(c->velocity.y) < 1.0f);

    std::cout << "  PASS: Restitution 0 rests on floor (no bounce)\n";
    std::cout << "    final pos.y=" << c->position.y << " vel.y=" << c->velocity.y << "\n";
}

void testOrbitStability() {
    World w(Vec2(0.0f, 0.0f), 1.0f / 120.0f);
    w.attractor.enabled = true;
    w.attractor.position = Vec2(0.0f, 0.0f);
    w.attractor.strength = 500000.0f;
    w.addBody(Vec2(0.0f, 0.0f), 0.0f, std::make_unique<CircleShape>(15.0f), 1.0f);
    RigidBody* orbiter = w.addBody(Vec2(150.0f, 0.0f), 1.0f, std::make_unique<CircleShape>(10.0f), 1.0f);
    orbiter->velocity = Vec2(0.0f, 57.74f);
    float maxR = 0.0f;
    bool nanDetected = false;
    for (int i = 0; i < 1200; ++i) {
        w.step();
        float r = orbiter->position.length();
        if (r > maxR) maxR = r;
        if (std::isnan(orbiter->position.x) || std::isnan(orbiter->position.y)) nanDetected = true;
    }
    CHECK(!nanDetected);
    CHECK(maxR < 400.0f);
    CHECK(maxR > 50.0f);
    std::cout << "  PASS: Orbit stability (maxR=" << maxR << ", no NaN)\n";
}

void testRadialImpulseMomentum() {
    World w(Vec2(0.0f, 0.0f), 1.0f / 120.0f);
    RigidBody* a = w.addBody(Vec2(100.0f, 0.0f), 2.0f, std::make_unique<CircleShape>(10.0f), 0.5f);
    RigidBody* b = w.addBody(Vec2(-100.0f, 0.0f), 3.0f, std::make_unique<CircleShape>(10.0f), 0.5f);
    w.applyRadialImpulse(Vec2(0.0f, 0.0f), 200.0f);
    CHECK(!std::isnan(a->velocity.x) && !std::isnan(a->velocity.y));
    CHECK(!std::isnan(b->velocity.x) && !std::isnan(b->velocity.y));
    CHECK(a->velocity.x > 0.0f);
    CHECK(b->velocity.x < 0.0f);
    CHECK(std::fabs(a->velocity.length() - 200.0f) < 0.01f);
    CHECK(std::fabs(b->velocity.length() - 200.0f) < 0.01f);
    std::cout << "  PASS: Radial impulse pushes bodies outward (vA=" << a->velocity.x << ", vB=" << b->velocity.x << ")\n";
}

void testRemoveBody() {
    World w(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    addFloor(w);
    w.addBody(Vec2(0.0f, 200.0f), 1.0f, std::make_unique<CircleShape>(10.0f), 0.5f);
    RigidBody* target = w.addBody(Vec2(100.0f, 100.0f), 2.0f, std::make_unique<CircleShape>(15.0f), 0.5f);
    w.addBody(Vec2(-50.0f, 50.0f), 1.5f, std::make_unique<CircleShape>(12.0f), 0.5f);
    CHECK(w.bodies.size() == 4);
    CHECK(w.removeBody(target));
    CHECK(w.bodies.size() == 3);
    CHECK(!w.removeBody(target));
    w.step();
    CHECK(w.bodies.size() == 3);
    std::cout << "  PASS: removeBody works (size=" << w.bodies.size() << ")\n";
}

void testClearBodies() {
    World w(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    addFloor(w);
    w.addBody(Vec2(0.0f, 200.0f), 1.0f, std::make_unique<CircleShape>(10.0f), 0.5f);
    w.addBody(Vec2(100.0f, 100.0f), 2.0f, std::make_unique<CircleShape>(15.0f), 0.5f);
    CHECK(w.bodies.size() == 3);
    w.clearBodies();
    CHECK(w.bodies.size() == 0);
    w.step();
    CHECK(w.bodies.size() == 0);
    std::cout << "  PASS: clearBodies empties world (size=" << w.bodies.size() << ")\n";
}

void testProfilerBasic() {
    FrameProfiler p;
    p.beginPhase("test");
    volatile float x = 0;
    for (int i = 0; i < 1000; ++i) x += 1.0f;
    p.endPhase();
    CHECK(p.getPhases().size() == 1);
    CHECK(p.getPhases()[0].microseconds > 0.0);
    CHECK(p.getTotalMicroseconds() > 0.0);
    std::cout << "  PASS: Profiler basic (phase=" << p.getPhases()[0].microseconds << " us)\n";
}

void testProfilerMultiplePhases() {
    FrameProfiler p;
    p.beginPhase("A"); p.endPhase();
    p.beginPhase("B"); p.endPhase();
    p.beginPhase("C"); p.endPhase();
    CHECK(p.getPhases().size() == 3);
    CHECK(p.getPhases()[0].name[0] == 'A');
    CHECK(p.getPhases()[1].name[0] == 'B');
    CHECK(p.getPhases()[2].name[0] == 'C');
    std::cout << "  PASS: Profiler multiple phases (total=" << p.getTotalMicroseconds() << " us)\n";
}

void testDetectorNaN() {
    InstabilityDetector d;
    d.checkBody(0, std::nanf(""), 0.0f, 0.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents().size() == 1);
    CHECK(d.getEvents()[0].type == InstabilityEvent::NaN_detected);
    std::cout << "  PASS: Detector catches NaN\n";
}

void testDetectorInf() {
    InstabilityDetector d;
    d.checkBody(1, 0.0f, std::numeric_limits<float>::infinity(), 0.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents()[0].type == InstabilityEvent::Inf_detected);
    std::cout << "  PASS: Detector catches Inf\n";
}

void testDetectorVelocityThreshold() {
    InstabilityDetector d;
    d.maxVelocity = 100.0f;
    d.checkBody(2, 0.0f, 0.0f, 500.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents()[0].type == InstabilityEvent::velocity_threshold);
    std::cout << "  PASS: Detector catches velocity threshold\n";
}

void testDetectorPositionThreshold() {
    InstabilityDetector d;
    d.maxPosition = 500.0f;
    d.checkBody(3, 1000.0f, 0.0f, 0.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents()[0].type == InstabilityEvent::position_threshold);
    std::cout << "  PASS: Detector catches position threshold\n";
}

void testDetectorCleanBody() {
    InstabilityDetector d;
    d.checkBody(4, 100.0f, -200.0f, 50.0f, -30.0f);
    CHECK(!d.hasEvents());
    std::cout << "  PASS: Detector clean body (no false positives)\n";
}

void testDetectorReset() {
    InstabilityDetector d;
    d.checkBody(0, std::nanf(""), 0.0f, 0.0f, 0.0f);
    CHECK(d.hasEvents());
    d.reset();
    CHECK(!d.hasEvents());
    std::cout << "  PASS: Detector reset clears events\n";
}

void testProfilerReset() {
    FrameProfiler p;
    p.beginPhase("A"); p.endPhase();
    CHECK(p.getPhases().size() == 1);
    p.reset();
    CHECK(p.getPhases().size() == 0);
    std::cout << "  PASS: Profiler reset clears phases\n";
}

void testWorldStepProfiles() {
    World w(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    addFloor(w);
    w.addBody(Vec2(0.0f, 200.0f), 1.0f, std::make_unique<CircleShape>(10.0f), 0.5f);
    w.step();
    CHECK(w.profiler.getPhases().size() == 5);
    CHECK(w.profiler.getTotalMicroseconds() > 0.0);
    CHECK(!w.detector.hasEvents());
    std::cout << "  PASS: World::step profiles 5 phases (total=" << w.profiler.getTotalMicroseconds() << " us)\n";
}

void testBrokenScenarioDetector() {
    World w(Vec2(0.0f, -9800000.0f), 1.0f / 120.0f);
    w.addBody(Vec2(0.0f, 200.0f), 1.0f, std::make_unique<CircleShape>(25.0f), 0.5f);
    for (int i = 0; i < 15; ++i) w.step();
    CHECK(w.detector.hasEvents());
    std::cout << "  PASS: Broken scenario triggers instability detector (" << w.detector.getEvents().size() << " events)\n";
}

struct BenchmarkResult {
    const char* name;
    double usPerStep;
    double resolveUsPerStep;
    double broadphaseUsPerStep;
    double narrowphaseUsPerStep;
    double solverUsPerStep;
    double correctionUsPerStep;
};

void benchmarkScenario(const char* name, int scenarioIndex, int steps, BenchmarkResult& out) {
    World w(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    w.fineProfileEnabled = true;
    loadScenario(w, scenarioIndex);
    for (int i = 0; i < 90; ++i) w.step();
    double total = 0.0;
    double resolveTotal = 0.0;
    double broadSum = 0.0, narrowSum = 0.0, solverSum = 0.0, corrSum = 0.0;
    for (int i = 0; i < steps; ++i) {
        w.step();
        total += w.profiler.getTotalMicroseconds();
        for (const auto& ph : w.profiler.getPhases()) {
            if (std::string(ph.name) == "Resolve Collisions") resolveTotal += ph.microseconds;
        }
        broadSum += w.fineBroadphaseUs;
        narrowSum += w.fineNarrowphaseUs;
        solverSum += w.fineSolverUs;
        corrSum += w.fineCorrectionUs;
    }
    out.name = name;
    out.usPerStep = total / steps;
    out.resolveUsPerStep = resolveTotal / steps;
    out.broadphaseUsPerStep = broadSum / steps;
    out.narrowphaseUsPerStep = narrowSum / steps;
    out.solverUsPerStep = solverSum / steps;
    out.correctionUsPerStep = corrSum / steps;
    std::cout << "    " << name << " (" << w.bodies.size() << " bodies): "
              << out.usPerStep << " us/step (resolve=" << out.resolveUsPerStep
              << " bp=" << out.broadphaseUsPerStep
              << " np=" << out.narrowphaseUsPerStep
              << " solver=" << out.solverUsPerStep
              << " corr=" << out.correctionUsPerStep << ")\n";
}

void benchmarkScenarioEx(const char* name, int scenarioIndex, int steps, bool brute) {
    World w(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    w.fineProfileEnabled = true;
    w.useBruteForceBroadPhase = brute;
    loadScenario(w, scenarioIndex);
    for (int i = 0; i < 90; ++i) w.step();
    double resolveSum = 0.0;
    double broadSum = 0.0, narrowSum = 0.0, solverSum = 0.0, corrSum = 0.0;
    for (int i = 0; i < steps; ++i) {
        w.step();
        for (const auto& ph : w.profiler.getPhases()) {
            if (std::string(ph.name) == "Resolve Collisions") resolveSum += ph.microseconds;
        }
        broadSum += w.fineBroadphaseUs;
        narrowSum += w.fineNarrowphaseUs;
        solverSum += w.fineSolverUs;
        corrSum += w.fineCorrectionUs;
    }
    std::cout << "    " << name << " [" << (brute ? "brute" : "grid") << "]: "
              << "resolve=" << resolveSum / steps << " us/step "
              << "broadphase=" << broadSum / steps << " us/step "
              << "narrowphase=" << narrowSum / steps << " us/step "
              << "solver=" << solverSum / steps << " us/step "
              << "correction=" << corrSum / steps << " us/step\n";
}

void runFinePhaseBreakdown() {
    std::cout << "\n  CHAOS PHASE BREAKDOWN (High-Restitution Chaos, 84 bodies)\n";
    benchmarkScenarioEx("Chaos", 7, 600, true);
    benchmarkScenarioEx("Chaos", 7, 600, false);
}

void runPerformanceBenchmarks(bool printOnly) {
    std::cout << "\n  PERFORMANCE BENCHMARKS\n";
    BenchmarkResult r[4];
    benchmarkScenario("Ball Pit (150+)", 3, 600, r[0]);
    benchmarkScenario("High-Restitution Chaos", 7, 600, r[1]);
    benchmarkScenario("Mixed Shapes Mosaic", 8, 600, r[2]);
    benchmarkScenario("Stress Ramp (150+)", 10, 600, r[3]);
    for (int i = 0; i < 4; ++i) {
        std::cout << "    " << r[i].name << ": " << r[i].usPerStep
                  << " us/step, resolve=" << r[i].resolveUsPerStep << " us\n";
    }
    if (!printOnly) {
        runFinePhaseBreakdown();
        return;
    }
}

void testGridMatchesBruteForce() {
    World gridWorld(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    World bruteWorld(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    bruteWorld.useBruteForceBroadPhase = true;

    auto buildScene = [](World& w) {
        w.addBody(Vec2(0.0f, -260.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), -270.0f, 0.0f));
        w.addBody(Vec2(-260.0f, 0.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(1.0f, 0.0f), -260.0f, 0.0f));
        w.addBody(Vec2(260.0f, 0.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(-1.0f, 0.0f), 260.0f, 0.0f));
        w.addBody(Vec2(0.0f, 260.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, -1.0f), 260.0f, 0.0f));
        for (int i = 0; i < 40; ++i) {
            float x = -230.0f + 12.0f * (i % 10) + (i % 5);
            float y = -230.0f + 12.0f * (i / 10) + (i % 3);
            w.addBody(Vec2(x, y), 1.0f, std::make_unique<CircleShape>(11.0f), 0.3f);
        }
        w.addBody(Vec2(200.0f, -200.0f), 2.0f,
                  std::make_unique<PolygonShape>(PolygonShape::makeBox(30.0f, 30.0f, 0.35f)), 0.4f);
        w.addBody(Vec2(-195.0f, 190.0f), 2.0f,
                  std::make_unique<PolygonShape>(PolygonShape::makeBox(28.0f, 36.0f)), 0.2f);
    };
    buildScene(gridWorld);
    buildScene(bruteWorld);

    for (int i = 0; i < 240; ++i) {
        gridWorld.step();
        bruteWorld.step();
    }

    for (size_t i = 0; i < gridWorld.bodies.size(); ++i) {
        const RigidBody& g = *gridWorld.bodies[i];
        const RigidBody& b = *bruteWorld.bodies[i];
        CHECK(g.position.x == b.position.x);
        CHECK(g.position.y == b.position.y);
        CHECK(g.velocity.x == b.velocity.x);
        CHECK(g.velocity.y == b.velocity.y);
        CHECK(g.angle == b.angle);
    }

    std::cout << "  PASS: Grid broad-phase matches brute force bit-for-bit (40 circles + 2 boxes + 4 planes, 240 steps)\n";
}

struct ScenarioCheck {
    int index;
    const char* name;
    int bodies;
    int steps;
    bool passed;
    const char* reason;
};

constexpr const char* SCENARIO_SUITE_OK = "";
constexpr const char* SCENARIO_SUITE_NAN = "NaN/Inf";
constexpr const char* SCENARIO_SUITE_ESCAPE = "escaped container";

void scenarioContainer(int index, float& halfW, float& halfH) {
    switch (index) {
    case 3:  halfW = 380.0f; halfH = 280.0f; break;
    case 7:  halfW = 350.0f; halfH = 260.0f; break;
    case 8:  halfW = 380.0f; halfH = 280.0f; break;
    case 9:  halfW = 380.0f; halfH = 280.0f; break;
    case 10: halfW = 380.0f; halfH = 280.0f; break;
    default: halfW = 0.0f;    halfH = 0.0f;    break;
    }
}

bool scenarioHasWalls(int index) {
    return index == 3 || index == 7 || index == 8 || index == 9 || index == 10;
}

bool scenarioBodyWithinBounds(const World& w, float halfW, float halfH) {
    float boundX = (halfW > 0.0f) ? halfW + 80.0f : 1500.0f;
    float boundY = (halfH > 0.0f) ? halfH + 80.0f : 1500.0f;
    for (const auto& body : w.bodies) {
        if (body->shape->getType() == ShapeType::Plane) continue;
        if (std::fabs(body->position.x) > boundX) return false;
        if (std::fabs(body->position.y) > boundY) return false;
    }
    return true;
}

int countDynamicBodies(const World& w) {
    int count = 0;
    for (const auto& body : w.bodies) {
        if (body->shape->getType() != ShapeType::Plane) ++count;
    }
    return count;
}

void densityBoost(World& w, int minDynamic, float halfW, float halfH) {
    float containerW = (halfW > 0.0f) ? halfW : 400.0f;
    float containerH = (halfH > 0.0f) ? halfH : 400.0f;
    int i = 0;
    while (countDynamicBodies(w) < minDynamic) {
        float x = -containerW + 20.0f + static_cast<float>(i % 15) * ((2.0f * containerW - 40.0f) / 15.0f);
        float y = containerH * 0.6f - static_cast<float>(i / 15) * 40.0f;
        float mass = 0.5f + static_cast<float>(i % 3) * 0.5f;
        float rest = 0.3f + static_cast<float>(i % 4) * 0.1f;
        w.addBody(Vec2(x, y), mass, std::make_unique<CircleShape>(8.0f + (i % 3) * 3.0f), rest);
        ++i;
    }
}

ScenarioCheck checkScenario(const Scenario& sc, int index, int steps, bool stress) {
    ScenarioCheck report;
    report.index = index;
    report.name = sc.name;
    report.steps = steps;
    report.passed = true;
    report.reason = SCENARIO_SUITE_OK;

    World w(Vec2(0.0f, -980.0f), 1.0f / 120.0f);
    sc.build(w);

    float halfW, halfH;
    scenarioContainer(index, halfW, halfH);
    if (stress) {
        if (!scenarioHasWalls(index)) {
            addContainerWalls(w, (halfW > 0.0f ? halfW : 400.0f),
                              (halfH > 0.0f ? halfH : 400.0f));
            halfW = (halfW > 0.0f) ? halfW : 400.0f;
            halfH = (halfH > 0.0f) ? halfH : 400.0f;
        }
        densityBoost(w, 150, halfW, halfH);
    }
    report.bodies = countDynamicBodies(w);

    for (int i = 0; i < steps; ++i) {
        w.step();
        if (w.detector.hasEvents()) {
            report.passed = false;
            report.reason = SCENARIO_SUITE_NAN;
            break;
        }
        if (!scenarioBodyWithinBounds(w, halfW, halfH)) {
            report.passed = false;
            report.reason = SCENARIO_SUITE_ESCAPE;
            break;
        }
    }
    return report;
}

int runScenarioSuite(bool stress) {
    std::cout << (stress ? "\n  HEADLESS SCENARIO SUITE (STRESS, 150+ bodies/scenario)\n"
                         : "\n  HEADLESS SCENARIO SUITE (nominal body counts)\n");
    const auto& list = getScenarioList();
    std::vector<ScenarioCheck> reports;
    int failures = 0;
    for (int idx = 0; idx < 11; ++idx) {
        ScenarioCheck r = checkScenario(list[idx], idx, 600, stress);
        reports.push_back(r);
        std::cout << "    " << (r.index + 1) << ". " << r.name
                  << "  bodies=" << r.bodies << " steps=" << r.steps
                  << "  [" << (r.passed ? "PASS" : "FAIL") << (r.passed ? "]" : " - " + std::string(r.reason) + "]") << "\n";
        if (!r.passed) ++failures;
    }
    std::cout << "    ---------------------------------------------------\n";
    if (failures == 0) {
        std::cout << "    All 11 scenarios passed.\n";
        return 0;
    }
    std::cout << "    " << failures << " scenario(s) FAILED.\n";
    return 1;
}

int main(int argc, char** argv) {
    std::string mode = "unit";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--scenario") mode = "scenario";
        else if (arg == "--stress") mode = "stress";
        else if (arg == "--phase") mode = "phase";
        else if (arg == "--unit") mode = "unit";
    }

    if (mode == "scenario") return runScenarioSuite(false);
    if (mode == "stress") return runScenarioSuite(true);
    if (mode == "phase") { runPerformanceBenchmarks(false); return 0; }

    std::cout << "ImpulseEngine Unit Tests\n";
    std::cout << "========================\n";

    testVec2Math();
    testSemiImplicitEulerFreeFall();
    testSemiImplicitEulerShortDuration();
    testStepCountTruncationRegression();
    testStaticBodyDoesNotMove();
    testPlaneCollisionDetection();
    testCirclePlaneImpulseKnownAnswer();
    testDropBouncesAndDoesNotSink();
    testRestingNoBounce();
    testCircleCircleDetection();
    testEqualMassElasticVelocitySwap();
    testMomentumConservedUnequalMass();
    testNonElasticCollisionLosesRelativeSpeed();
    testIdenticalPositionCirclesNoNaN();
    testPolygonVsPlaneDetectionAndImpulse();
    testPolygonPolygonDetection();
    testCirclePolygonDetection();
    testBoxStackStable();
    testFrictionStopsSlidingBlock();
    testFrictionRampOrdering();
    testBoxStack10Stable();
    testOrbitStability();
    testRadialImpulseMomentum();
    testRemoveBody();
    testClearBodies();
    testProfilerBasic();
    testProfilerMultiplePhases();
    testDetectorNaN();
    testDetectorInf();
    testDetectorVelocityThreshold();
    testDetectorPositionThreshold();
    testDetectorCleanBody();
    testDetectorReset();
    testProfilerReset();
    testWorldStepProfiles();
    testBrokenScenarioDetector();
    testGridMatchesBruteForce();
    runPerformanceBenchmarks(true);

    std::cout << "========================\n";
    std::cout << "All tests passed.\n";
    return 0;
}
