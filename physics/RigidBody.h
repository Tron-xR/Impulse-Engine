#pragma once

#include "Vec2.h"
#include <memory>
#include <vector>

enum class ShapeType {
    Circle,
    Polygon,
    Plane,
};

class Shape {
public:
    float friction = 0.3f;
    virtual ~Shape() = default;
    virtual ShapeType getType() const = 0;
};

class CircleShape : public Shape {
public:
    float radius;

    explicit CircleShape(float radius, float friction = 0.3f) : radius(radius) { this->friction = friction; }
    ShapeType getType() const override { return ShapeType::Circle; }
};

class PolygonShape : public Shape {
public:
    std::vector<Vec2> vertices;
    float restitution;

    PolygonShape(std::vector<Vec2> verts, float restitution = 0.3f, float friction = 0.5f)
        : vertices(std::move(verts))
        , restitution(restitution)
    { this->friction = friction; }

    static PolygonShape makeBox(float halfWidth, float halfHeight, float restitution = 0.3f, float friction = 0.5f) {
        std::vector<Vec2> verts;
        verts.push_back({ -halfWidth, -halfHeight });
        verts.push_back({  halfWidth, -halfHeight });
        verts.push_back({  halfWidth,  halfHeight });
        verts.push_back({ -halfWidth,  halfHeight });
        return PolygonShape(std::move(verts), restitution, friction);
    }

    ShapeType getType() const override { return ShapeType::Polygon; }
};

class PlaneShape : public Shape {
public:
    Vec2 normal;
    float offset;
    float restitution;

    PlaneShape(const Vec2& normal, float offset, float restitution = 0.3f, float friction = 0.6f)
        : normal(normal.normalized())
        , offset(offset)
        , restitution(restitution)
    { this->friction = friction; }

    ShapeType getType() const override { return ShapeType::Plane; }
};

class RigidBody {
public:
    Vec2 position;
    Vec2 velocity;
    Vec2 force;

    float angle = 0.0f;
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
