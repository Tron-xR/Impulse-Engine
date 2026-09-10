#pragma once

#include "physics/RigidBody.h"
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

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

bool detectPolygonVsPlane(const RigidBody& polyBody, const PlaneShape& plane, Manifold& out) {
    if (polyBody.shape->getType() != ShapeType::Polygon) return false;

    const PolygonShape& poly = static_cast<const PolygonShape&>(*polyBody.shape);

    float minDistance = std::numeric_limits<float>::max();
    Vec2 deepestVertex;
    for (size_t i = 0; i < poly.vertices.size(); ++i) {
        Vec2 world = poly.vertices[i].rotated(polyBody.angle) + polyBody.position;
        float distance = world.dot(plane.normal) - plane.offset;
        if (distance < minDistance) {
            minDistance = distance;
            deepestVertex = world;
        }
    }
    if (minDistance >= 0.0f) return false;

    out.normal = plane.normal;
    out.penetration = -minDistance;
    out.contactPoint = deepestVertex;
    out.restitution = polyBody.restitution > plane.restitution ? polyBody.restitution : plane.restitution;
    return true;
}

namespace {
std::vector<Vec2> worldVertices(const PolygonShape& poly, const RigidBody& body) {
    std::vector<Vec2> verts;
    verts.reserve(poly.vertices.size());
    for (const Vec2& v : poly.vertices) {
        verts.push_back(v.rotated(body.angle) + body.position);
    }
    return verts;
}

Vec2 faceOutwardNormal(const std::vector<Vec2>& verts, size_t i) {
    Vec2 edge = verts[(i + 1) % verts.size()] - verts[i];
    return Vec2(edge.y, -edge.x).normalized();
}

Vec2 closestPointOnSegment(const Vec2& a, const Vec2& b, const Vec2& p) {
    Vec2 ab = b - a;
    float lenSq = ab.lengthSq();
    if (lenSq < 1e-12f) return a;
    float t = (p - a).dot(ab) / lenSq;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return a + ab * t;
}
}

bool detectPolygonPolygon(const RigidBody& a, const RigidBody& b, Manifold& out) {
    if (a.shape->getType() != ShapeType::Polygon || b.shape->getType() != ShapeType::Polygon) return false;

    const PolygonShape& pa = static_cast<const PolygonShape&>(*a.shape);
    const PolygonShape& pb = static_cast<const PolygonShape&>(*b.shape);
    std::vector<Vec2> va = worldVertices(pa, a);
    std::vector<Vec2> vb = worldVertices(pb, b);

    auto project = [](const std::vector<Vec2>& verts, const Vec2& axis) {
        float mn = std::numeric_limits<float>::max();
        float mx = -std::numeric_limits<float>::max();
        for (const Vec2& v : verts) {
            float d = v.dot(axis);
            mn = std::min(mn, d);
            mx = std::max(mx, d);
        }
        return std::pair<float, float>(mn, mx);
    };

    auto overlapOn = [&](const Vec2& axis) {
        std::pair<float, float> ap = project(va, axis);
        std::pair<float, float> bp = project(vb, axis);
        return std::min(ap.second, bp.second) - std::max(ap.first, bp.first);
    };

    float minOverlap = std::numeric_limits<float>::max();
    Vec2 minAxis;

    for (size_t i = 0; i < va.size(); ++i) {
        Vec2 axis = faceOutwardNormal(va, i);
        float overlap = overlapOn(axis);
        if (overlap <= 0.0f) return false;
        if (overlap < minOverlap) { minOverlap = overlap; minAxis = axis; }
    }
    for (size_t i = 0; i < vb.size(); ++i) {
        Vec2 axis = faceOutwardNormal(vb, i);
        float overlap = overlapOn(axis);
        if (overlap <= 0.0f) return false;
        if (overlap < minOverlap) { minOverlap = overlap; minAxis = axis; }
    }

    if (minAxis.dot(a.position - b.position) < 0.0f) minAxis = -minAxis;

    out.normal = minAxis;
    out.penetration = minOverlap;
    out.contactPoint = (a.position + b.position) * 0.5f;
    out.restitution = a.restitution > b.restitution ? a.restitution : b.restitution;
    return true;
}

bool detectCirclePolygon(const RigidBody& circleBody, const RigidBody& polyBody, Manifold& out) {
    if (circleBody.shape->getType() != ShapeType::Circle || polyBody.shape->getType() != ShapeType::Polygon) return false;

    const CircleShape& circle = static_cast<const CircleShape&>(*circleBody.shape);
    const PolygonShape& poly = static_cast<const PolygonShape&>(*polyBody.shape);
    std::vector<Vec2> verts = worldVertices(poly, polyBody);
    const Vec2 center = circleBody.position;

    float maxFaceDistance = -std::numeric_limits<float>::max();
    size_t maxFace = 0;
    for (size_t i = 0; i < verts.size(); ++i) {
        Vec2 n = faceOutwardNormal(verts, i);
        float d = (center - verts[i]).dot(n);
        if (d > maxFaceDistance) { maxFaceDistance = d; maxFace = i; }
    }

    if (maxFaceDistance <= 0.0f) {
        out.normal = faceOutwardNormal(verts, maxFace);
        out.penetration = circle.radius + maxFaceDistance;
        if (out.penetration <= 0.0f) return false;
        out.contactPoint = center - out.normal * maxFaceDistance;
    } else {
        Vec2 closest = verts[0];
        float bestDistSq = std::numeric_limits<float>::max();
        for (size_t i = 0; i < verts.size(); ++i) {
            Vec2 p = closestPointOnSegment(verts[i], verts[(i + 1) % verts.size()], center);
            float distSq = (center - p).lengthSq();
            if (distSq < bestDistSq) { bestDistSq = distSq; closest = p; }
        }
        float distance = std::sqrt(bestDistSq);
        if (distance >= circle.radius) return false;
        out.normal = distance > 1e-6f ? (center - closest) / distance : Vec2(0.0f, 1.0f);
        out.penetration = circle.radius - distance;
        out.contactPoint = closest;
    }

    out.restitution = circleBody.restitution > polyBody.restitution ? circleBody.restitution : polyBody.restitution;
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

    constexpr float RESTITUTION_VELOCITY_THRESHOLD = 30.0f;
    float e = manifold.restitution;
    if (velocityAlongNormal > -RESTITUTION_VELOCITY_THRESHOLD) e = 0.0f;

    float j = -(1.0f + e) * velocityAlongNormal / invMassSum;
    Vec2 impulse = manifold.normal * j;
    a.velocity += impulse * a.invMass;
    b.velocity -= impulse * b.invMass;
}

void applyPositionalCorrection(RigidBody& a, RigidBody& b, const Manifold& manifold,
                               float percent, float slop) {
    float invMassSum = a.invMass + b.invMass;
    if (invMassSum == 0.0f) return;

    float correction = std::max(manifold.penetration - slop, 0.0f) / invMassSum * percent;
    if (correction <= 0.0f) return;

    Vec2 displacement = manifold.normal * correction;
    a.position += displacement * a.invMass;
    b.position -= displacement * b.invMass;
}