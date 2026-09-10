#pragma once

#include "RigidBody.h"
#include "../collision/CollisionDetect.h"
#include "../diagnostics/Diagnostics.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

struct GravityWell {
    Vec2 position = { 0.0f, 0.0f };
    float strength = 0.0f;
    bool enabled = false;
};

class World {
public:
    std::vector<std::unique_ptr<RigidBody>> bodies;
    Vec2 gravity;
    float fixedDt;
    int solverIterations = 15;
    float positionCorrectionPercent = 1.0f;
    float positionSlop = 0.01f;
    GravityWell attractor;
    FrameProfiler profiler;
    InstabilityDetector detector;

    World(const Vec2& gravity = Vec2(0.0f, -980.0f), float fixedDt = 1.0f / 120.0f)
        : gravity(gravity)
        , fixedDt(fixedDt)
    {}

    RigidBody* addBody(const Vec2& pos, float mass, std::unique_ptr<Shape> shape, float restitution = 0.3f) {
        bodies.push_back(std::make_unique<RigidBody>(pos, mass, std::move(shape), restitution));
        return bodies.back().get();
    }

    bool removeBody(RigidBody* ptr) {
        auto it = std::find_if(bodies.begin(), bodies.end(),
                               [ptr](const std::unique_ptr<RigidBody>& b) { return b.get() == ptr; });
        if (it == bodies.end()) return false;
        bodies.erase(it);
        return true;
    }

    void clearBodies() { bodies.clear(); }

    void applyRadialImpulse(const Vec2& center, float strength) {
        for (auto& body : bodies) {
            if (body->invMass == 0.0f) continue;
            Vec2 dir = body->position - center;
            float dist = dir.length();
            if (dist < 1e-4f) continue;
            body->velocity += (dir / dist) * strength;
        }
    }

    void step() {
        profiler.reset();
        detector.reset();

        profiler.beginPhase("Apply Forces");
        for (auto& body : bodies) {
            if (body->invMass == 0.0f) continue;
            body->applyForce(gravity * body->mass);
            if (attractor.enabled) {
                Vec2 dir = attractor.position - body->position;
                float distSq = dir.lengthSq();
                if (distSq > 1e-8f) {
                    float dist = std::sqrt(distSq);
                    body->applyForce((dir / dist) * attractor.strength * body->mass / distSq);
                }
            }
        }
        profiler.endPhase();

        profiler.beginPhase("Integrate Velocity");
        for (auto& body : bodies) {
            if (body->invMass == 0.0f) continue;
            body->velocity += body->force * body->invMass * fixedDt;
            body->clearForces();
        }
        profiler.endPhase();

        profiler.beginPhase("Resolve Collisions");
        resolveCollisions();
        profiler.endPhase();

        profiler.beginPhase("Integrate Position");
        for (auto& body : bodies) {
            if (body->invMass == 0.0f) continue;
            body->position += body->velocity * fixedDt;
        }
        profiler.endPhase();

        profiler.beginPhase("Detect Instability");
        for (size_t i = 0; i < bodies.size(); ++i) {
            auto& body = bodies[i];
            detector.checkBody(static_cast<int>(i), body->position.x, body->position.y,
                               body->velocity.x, body->velocity.y);
        }
        profiler.endPhase();
    }

private:
    struct PairContact {
        RigidBody* a = nullptr;
        RigidBody* b = nullptr;
        Manifold manifold;
        bool contact = false;
        float normalImpulseAccum = 0.0f;
    };

    static const PlaneShape& asPlane(const RigidBody& body) {
        return static_cast<const PlaneShape&>(*body.shape);
    }

    PairContact detectPair(RigidBody& x, RigidBody& y) const {
        PairContact result;
        ShapeType tx = x.shape->getType();
        ShapeType ty = y.shape->getType();

        if (tx == ShapeType::Circle && ty == ShapeType::Plane) {
            result.a = &x; result.b = &y;
            result.contact = detectCircleVsPlane(x, asPlane(y), result.manifold);
        } else if (tx == ShapeType::Plane && ty == ShapeType::Circle) {
            result.a = &y; result.b = &x;
            result.contact = detectCircleVsPlane(y, asPlane(x), result.manifold);
        } else if (tx == ShapeType::Circle && ty == ShapeType::Circle) {
            result.a = &x; result.b = &y;
            result.contact = detectCircleCircle(x, y, result.manifold);
        } else if (tx == ShapeType::Circle && ty == ShapeType::Polygon) {
            result.a = &x; result.b = &y;
            result.contact = detectCirclePolygon(x, y, result.manifold);
        } else if (tx == ShapeType::Polygon && ty == ShapeType::Circle) {
            result.a = &y; result.b = &x;
            result.contact = detectCirclePolygon(y, x, result.manifold);
        } else if (tx == ShapeType::Polygon && ty == ShapeType::Plane) {
            result.a = &x; result.b = &y;
            result.contact = detectPolygonVsPlane(x, asPlane(y), result.manifold);
        } else if (tx == ShapeType::Plane && ty == ShapeType::Polygon) {
            result.a = &y; result.b = &x;
            result.contact = detectPolygonVsPlane(y, asPlane(x), result.manifold);
        } else if (tx == ShapeType::Polygon && ty == ShapeType::Polygon) {
            result.a = &x; result.b = &y;
            result.contact = detectPolygonPolygon(x, y, result.manifold);
        }
        return result;
    }

    void resolveCollisions() {
        std::vector<PairContact> state;
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                PairContact pc = detectPair(*bodies[i], *bodies[j]);
                if (pc.contact) {
                    state.push_back(pc);
                }
            }
        }

        for (int iteration = 0; iteration < solverIterations; ++iteration) {
            for (PairContact& pc : state) {
                if (pc.a->invMass == 0.0f && pc.b->invMass == 0.0f) continue;
                pc.normalImpulseAccum += applyImpulse(*pc.a, *pc.b, pc.manifold);
                applyFrictionImpulse(*pc.a, *pc.b, pc.manifold, pc.normalImpulseAccum);
            }
        }

        for (PairContact& pc : state) {
            if (pc.a->invMass == 0.0f && pc.b->invMass == 0.0f) continue;
            applyFrictionImpulse(*pc.a, *pc.b, pc.manifold, pc.normalImpulseAccum);
        }

        for (int p = 0; p < 3; ++p) {
            for (size_t i = 0; i < bodies.size(); ++i) {
                for (size_t j = i + 1; j < bodies.size(); ++j) {
                    PairContact pc = detectPair(*bodies[i], *bodies[j]);
                    if (pc.contact) {
                        applyPositionalCorrection(*pc.a, *pc.b, pc.manifold,
                                                  positionCorrectionPercent, positionSlop);
                    }
                }
            }
        }
    }
};