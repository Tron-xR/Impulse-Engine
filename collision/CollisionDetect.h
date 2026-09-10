#pragma once

#include "physics/RigidBody.h"

struct Manifold {
    Vec2 normal;
    float penetration;
    Vec2 contactPoint;
    float restitution;
};

bool detectCircleVsPlane(const RigidBody& circleBody, const PlaneShape& plane, Manifold& out) {
    if (circleBody.shape->getType() != ShapeType::Circle) return false;

    const CircleShape& circle = static_cast<const CircleShape&>(*circleBody.shape);
    float distance = circleBody.position.dot(plane.normal) - plane.offset;
    if (distance >= circle.radius) return false;

    out.normal = plane.normal;
    out.penetration = circle.radius - distance;
    out.contactPoint = circleBody.position - plane.normal * distance;
    out.restitution = circleBody.restitution > plane.restitution ? circleBody.restitution : plane.restitution;
    return true;
}

bool detectCircleCircle(const RigidBody& a, const RigidBody& b, Manifold& out) {
    if (a.shape->getType() != ShapeType::Circle || b.shape->getType() != ShapeType::Circle) return false;

    const CircleShape& ca = static_cast<const CircleShape&>(*a.shape);
    const CircleShape& cb = static_cast<const CircleShape&>(*b.shape);

    Vec2 delta = a.position - b.position;
    float radiusSum = ca.radius + cb.radius;
    float distSq = delta.lengthSq();
    if (distSq >= radiusSum * radiusSum) return false;

    if (distSq < 1e-8f) {
        out.normal = Vec2(0.0f, 1.0f);
        out.penetration = radiusSum;
        out.contactPoint = a.position;
    } else {
        float dist = std::sqrt(distSq);
        out.normal = delta / dist;
        out.penetration = radiusSum - dist;
        out.contactPoint = (a.position + b.position) * 0.5f;
    }
    out.restitution = a.restitution > b.restitution ? a.restitution : b.restitution;
    return true;
}

void applyImpulse(RigidBody& a, RigidBody& b, const Manifold& manifold) {
    Vec2 relativeVelocity = a.velocity - b.velocity;
    float velocityAlongNormal = relativeVelocity.dot(manifold.normal);
    if (velocityAlongNormal >= 0.0f) return;

    float invMassSum = a.invMass + b.invMass;
    if (invMassSum == 0.0f) return;

    float j = -(1.0f + manifold.restitution) * velocityAlongNormal / invMassSum;
    Vec2 impulse = manifold.normal * j;
    a.velocity += impulse * a.invMass;
    b.velocity -= impulse * b.invMass;
}