#pragma once

#include "Vec2.h"
#include <memory>

enum class ShapeType {
    Circle,
};

class Shape {
public:
    virtual ~Shape() = default;
    virtual ShapeType getType() const = 0;
};

class CircleShape : public Shape {
public:
    float radius;

    explicit CircleShape(float radius) : radius(radius) {}
    ShapeType getType() const override { return ShapeType::Circle; }
};

class RigidBody {
public:
    Vec2 position;
    Vec2 velocity;
    Vec2 force;

    float mass;
    float invMass;
    float restitution;

    std::unique_ptr<Shape> shape;

    RigidBody(const Vec2& pos, float mass, std::unique_ptr<Shape> shape, float restitution = 0.3f)
        : position(pos)
        , velocity(0.0f, 0.0f)
        , force(0.0f, 0.0f)
        , mass(mass)
        , invMass(mass > 0.0f ? 1.0f / mass : 0.0f)
        , restitution(restitution)
        , shape(std::move(shape))
    {}

    void applyForce(const Vec2& f) { force += f; }

    void clearForces() { force = { 0.0f, 0.0f }; }
};
