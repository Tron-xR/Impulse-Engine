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
    void resolveCollisions() {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                RigidBody* circle = nullptr;
                RigidBody* plane = nullptr;
                if (bodies[i]->shape->getType() == ShapeType::Circle && bodies[j]->shape->getType() == ShapeType::Plane) {
                    circle = bodies[i].get();
                    plane = bodies[j].get();
                } else if (bodies[i]->shape->getType() == ShapeType::Plane && bodies[j]->shape->getType() == ShapeType::Circle) {
                    circle = bodies[j].get();
                    plane = bodies[i].get();
                }

                if (circle && plane) {
                    if (circle->invMass == 0.0f) continue;
                    Manifold manifold;
                    const PlaneShape& planeShape = static_cast<const PlaneShape&>(*plane->shape);
                    if (detectCircleVsPlane(*circle, planeShape, manifold)) {
                        applyImpulse(*circle, *plane, manifold);
                    }
                    continue;
                }

                if (bodies[i]->shape->getType() == ShapeType::Circle && bodies[j]->shape->getType() == ShapeType::Circle) {
                    if (bodies[i]->invMass == 0.0f && bodies[j]->invMass == 0.0f) continue;
                    Manifold manifold;
                    if (detectCircleCircle(*bodies[i], *bodies[j], manifold)) {
                        applyImpulse(*bodies[i], *bodies[j], manifold);
                    }
                }
            }
        }
    }
};