#pragma once

#include "RigidBody.h"
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
            body->position += body->velocity * fixedDt;
            body->clearForces();
        }
    }
};
