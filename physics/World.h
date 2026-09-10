#pragma once

#include "RigidBody.h"
#include "../collision/CollisionDetect.h"
#include <vector>
#include <memory>

class World {
public:
    std::vector<std::unique_ptr<RigidBody>> bodies;
    Vec2 gravity;
    float fixedDt;
    int solverIterations = 4;
    float positionCorrectionPercent = 0.8f;
    float positionSlop = 0.01f;

    World(const Vec2& gravity = Vec2(0.0f, -980.0f), float fixedDt = 1.0f / 120.0f)
        : gravity(gravity)
        , fixedDt(fixedDt)
    {}

    RigidBody* addBody(const Vec2& pos, float mass, std::unique_ptr<Shape> shape, float restitution = 0.3f) {
        bodies.push_back(std::make_unique<RigidBody>(pos, mass, std::move(shape), restitution));
        return bodies.back().get();
    }

    void step() {
        for (auto& body : bodies) {
            if (body->invMass == 0.0f) continue;
            body->applyForce(gravity * body->mass);
        }

        for (auto& body : bodies) {
            if (body->invMass == 0.0f) continue;
            body->velocity += body->force * body->invMass * fixedDt;
            body->clearForces();
        }

        resolveCollisions();

        for (auto& body : bodies) {
            if (body->invMass == 0.0f) continue;
            body->position += body->velocity * fixedDt;
        }
    }

private:
    struct PairContact {
        RigidBody* a = nullptr;
        RigidBody* b = nullptr;
        Manifold manifold;
        bool contact = false;
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
        for (int iteration = 0; iteration < solverIterations; ++iteration) {
            for (size_t i = 0; i < bodies.size(); ++i) {
                for (size_t j = i + 1; j < bodies.size(); ++j) {
                    PairContact pc = detectPair(*bodies[i], *bodies[j]);
                    if (pc.contact) {
                        applyImpulse(*pc.a, *pc.b, pc.manifold);
                    }
                }
            }
        }

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
};