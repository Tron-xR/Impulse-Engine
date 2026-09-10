#include <cmath>
#include <iostream>
#include <cstdlib>
#include <algorithm>

#include "physics/Vec2.h"
#include "physics/RigidBody.h"
#include "physics/World.h"
#include "collision/CollisionDetect.h"

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

int main() {
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

    std::cout << "========================\n";
    std::cout << "All tests passed.\n";
    return 0;
}
