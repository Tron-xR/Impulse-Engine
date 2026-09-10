#pragma once

#include "physics/World.h"
#include <functional>
#include <vector>
#include <string>
#include <cmath>

struct Scenario {
    const char* name;
    std::function<void(World&)> build;
};

inline void addContainerWalls(World& world, float halfW, float halfH, float restitution = 0.3f, float friction = 0.6f) {
    float floorY = -halfH;
    float ceilY = halfH;
    float leftX = -halfW;
    float rightX = halfW;
    world.addBody(Vec2(0.0f, floorY - 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), floorY, restitution, friction));
    world.addBody(Vec2(0.0f, ceilY + 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, -1.0f), -ceilY, restitution, friction));
    world.addBody(Vec2(leftX - 10.0f, 0.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(1.0f, 0.0f), leftX, restitution, friction));
    world.addBody(Vec2(rightX + 10.0f, 0.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(-1.0f, 0.0f), -rightX, restitution, friction));
}

inline void addFloor(World& world, float floorY = -220.0f, float restitution = 0.3f, float friction = 0.6f) {
    world.addBody(Vec2(0.0f, floorY - 10.0f), 0.0f,
                  std::make_unique<PlaneShape>(Vec2(0.0f, 1.0f), floorY, restitution, friction));
}

inline void addBoxStack(World& world, int count, float halfSize, float floorY,
                        float restitution = 0.0f, float friction = 0.5f) {
    for (int i = 0; i < count; ++i) {
        world.addBody(Vec2(0.0f, floorY + halfSize + 2.0f * halfSize * i), 1.0f,
                      std::make_unique<PolygonShape>(PolygonShape::makeBox(halfSize, halfSize, restitution, friction)),
                      restitution);
    }
}

inline void addBallRing(World& world, int count, float centerRadius, float ballRadius,
                        float mass, float restitution, Vec2 center) {
    for (int i = 0; i < count; ++i) {
        float angle = 6.2831853f * i / count;
        Vec2 pos = center + Vec2(std::cos(angle) * centerRadius, std::sin(angle) * centerRadius);
        world.addBody(pos, mass, std::make_unique<CircleShape>(ballRadius), restitution);
    }
}

namespace scenarios {

inline void emptyWorld(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addFloor(world, -220.0f, 0.3f, 0.6f);
}

inline void singleBounce(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addFloor(world, -220.0f, 0.5f, 0.6f);
    world.addBody(Vec2(0.0f, 200.0f), 2.0f, std::make_unique<CircleShape>(25.0f), 0.8f);
}

inline void boxStack(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addFloor(world, -220.0f, 0.0f, 0.6f);
    addBoxStack(world, 10, 25.0f, -220.0f);
}

inline void ballPit(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addContainerWalls(world, 380.0f, 280.0f, 0.3f, 0.4f);
    for (int i = 0; i < 150; ++i) {
        float x = -350.0f + static_cast<float>(i % 15) * 48.0f;
        float y = 50.0f + static_cast<float>(i / 15) * 48.0f;
        float r = 8.0f + (i % 5) * 3.0f;
        float mass = 0.5f + (i % 4) * 0.5f;
        float rest = 0.4f + (i % 3) * 0.2f;
        world.addBody(Vec2(x, y), mass, std::make_unique<CircleShape>(r), rest);
    }
}

inline void pyramid(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addFloor(world, -220.0f, 0.0f, 0.6f);
    float halfSize = 22.0f;
    float floorY = -220.0f;
    for (int row = 0; row < 6; ++row) {
        int count = 6 - row;
        float baseY = floorY + halfSize + 2.0f * halfSize * row;
        for (int col = 0; col < count; ++col) {
            float x = -(count - 1) * halfSize + col * 2.0f * halfSize;
            world.addBody(Vec2(x, baseY), 1.0f,
                          std::make_unique<PolygonShape>(PolygonShape::makeBox(halfSize, halfSize, 0.0f, 0.6f)),
                          0.0f);
        }
    }
}

inline void orbitingBodies(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, 0.0f);
    world.attractor.enabled = true;
    world.attractor.position = Vec2(0.0f, 0.0f);
    world.attractor.strength = 500000.0f;

    world.addBody(Vec2(0.0f, 0.0f), 0.0f, std::make_unique<CircleShape>(15.0f, 0.3f), 1.0f);

    float orbits[][3] = {
        { 150.0f, 0.0f, 57.74f },
        { -200.0f, 0.0f, 50.0f },
        { 0.0f, 250.0f, 44.72f },
    };
    for (auto& o : orbits) {
        RigidBody* b = world.addBody(Vec2(o[0], o[1]), 1.0f,
                                     std::make_unique<CircleShape>(10.0f), 1.0f);
        b->velocity = Vec2(-o[2], 0.0f) * (o[1] != 0.0f ? 1.0f : 1.0f);
        if (std::fabs(o[0]) > 1.0f) {
            b->velocity = Vec2(0.0f, o[2] * (o[0] > 0.0f ? 1.0f : -1.0f));
        } else {
            b->velocity = Vec2(o[2] * (o[1] > 0.0f ? -1.0f : 1.0f), 0.0f);
        }
    }
}

inline void frictionRamp(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    float slopeAngle = 0.35f;
    Vec2 slopeNormal(std::sin(slopeAngle), std::cos(slopeAngle));
    float slopeOffset = -180.0f;
    world.addBody(Vec2(0.0f, -300.0f), 0.0f,
                  std::make_unique<PlaneShape>(slopeNormal, slopeOffset, 0.0f, 0.8f));
    float frictions[] = { 0.05f, 0.2f, 0.5f, 1.0f };
    Vec2 alongSlope(slopeNormal.y, -slopeNormal.x);
    for (int i = 0; i < 4; ++i) {
        float t = slopeOffset + 20.0f + 50.0f * i;
        Vec2 pos = slopeNormal * (-t) + alongSlope * (-100.0f + 60.0f * i);
        pos.y += 50.0f;
        RigidBody* b = world.addBody(pos, 2.0f,
                                     std::make_unique<PolygonShape>(PolygonShape::makeBox(15.0f, 15.0f, 0.0f, frictions[i])),
                                     0.0f);
        b->velocity = alongSlope * 120.0f;
    }
}

inline void highRestitutionChaos(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addContainerWalls(world, 350.0f, 260.0f, 0.95f, 0.1f);
    for (int i = 0; i < 80; ++i) {
        float x = -320.0f + static_cast<float>(i % 10) * 64.0f;
        float y = 30.0f + static_cast<float>(i / 10) * 55.0f;
        float r = 10.0f + (i % 4) * 5.0f;
        float mass = 0.5f + (i % 3) * 1.0f;
        RigidBody* b = world.addBody(Vec2(x, y), mass,
                                     std::make_unique<CircleShape>(r), 0.95f);
        b->velocity = Vec2(100.0f * ((i % 2) ? 1.0f : -1.0f),
                           80.0f * ((i % 3) ? 1.0f : -1.0f));
    }
}

inline void mixedShapesMosaic(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addContainerWalls(world, 380.0f, 280.0f, 0.4f, 0.5f);
    for (int i = 0; i < 100; ++i) {
        float x = -340.0f + static_cast<float>(i % 10) * 68.0f;
        float y = 50.0f + static_cast<float>(i / 10) * 50.0f;
        float mass = 0.5f + (i % 5) * 0.5f;
        float rest = 0.3f + (i % 4) * 0.15f;
        if (i % 3 == 0) {
            float halfW = 10.0f + (i % 4) * 5.0f;
            world.addBody(Vec2(x, y), mass,
                          std::make_unique<PolygonShape>(PolygonShape::makeBox(halfW, halfW, rest, 0.5f)),
                          rest);
        } else {
            float r = 8.0f + (i % 5) * 4.0f;
            world.addBody(Vec2(x, y), mass,
                          std::make_unique<CircleShape>(r), rest);
        }
    }
}

inline void explosion(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addContainerWalls(world, 380.0f, 280.0f, 0.3f, 0.6f);
    for (int i = 0; i < 60; ++i) {
        float angle = 6.2831853f * i / 60.0f;
        float r = 80.0f + (i % 3) * 30.0f;
        Vec2 pos(std::cos(angle) * r, std::sin(angle) * r);
        world.addBody(pos, 1.0f + (i % 3) * 0.5f,
                      std::make_unique<CircleShape>(10.0f + (i % 4) * 3.0f), 0.5f);
    }
    world.applyRadialImpulse(Vec2(0.0f, 0.0f), 300.0f);
}

inline void stressRamp(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -980.0f);
    world.attractor = GravityWell();
    addContainerWalls(world, 380.0f, 280.0f, 0.3f, 0.4f);
    for (int i = 0; i < 150; ++i) {
        float x = -350.0f + static_cast<float>(i % 15) * 48.0f;
        float y = 50.0f + static_cast<float>(i / 15) * 48.0f;
        world.addBody(Vec2(x, y), 1.0f,
                      std::make_unique<CircleShape>(8.0f), 0.7f);
    }
}

inline void brokenExtremeVelocity(World& world) {
    world.clearBodies();
    world.gravity = Vec2(0.0f, -9800000.0f);
    world.attractor = GravityWell();
    addFloor(world, -220.0f, 0.0f, 0.6f);
    world.addBody(Vec2(0.0f, 200.0f), 1.0f, std::make_unique<CircleShape>(25.0f), 0.5f);
}

} // namespace scenarios

inline const std::vector<Scenario>& getScenarioList() {
    static const std::vector<Scenario> list = {
        { "1. Empty World",          scenarios::emptyWorld },
        { "2. Single Bounce",        scenarios::singleBounce },
        { "3. Box Stack",            scenarios::boxStack },
        { "4. Ball Pit (150+)",      scenarios::ballPit },
        { "5. Pyramid",              scenarios::pyramid },
        { "6. Orbiting Bodies",      scenarios::orbitingBodies },
        { "7. Friction Ramp",        scenarios::frictionRamp },
        { "8. High-Restitution Chaos", scenarios::highRestitutionChaos },
        { "9. Mixed Shapes Mosaic",  scenarios::mixedShapesMosaic },
        { "10. Explosion",           scenarios::explosion },
        { "11. Stress Ramp (150+)",  scenarios::stressRamp },
        { "12. [BROKEN] Extreme Velocity", scenarios::brokenExtremeVelocity },
    };
    return list;
}

inline void loadScenario(World& world, int index) {
    const auto& list = getScenarioList();
    if (index >= 0 && index < static_cast<int>(list.size())) {
        list[index].build(world);
    }
}
