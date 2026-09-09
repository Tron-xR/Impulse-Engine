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

void applyImpulse(RigidBody& body, const Manifold& manifold) {
    float velAlongNormal = body.velocity.dot(manifold.normal);
    if (velAlongNormal >= 0.0f) return;

    float invMassSum = body.invMass;
    if (invMassSum == 0.0f) return;

    float j = -(1.0f + manifold.restitution) * velAlongNormal / invMassSum;
    body.velocity += manifold.normal * (j * body.invMass);
}