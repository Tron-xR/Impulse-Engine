#pragma once

#include "RigidBody.h"
#include "../collision/CollisionDetect.h"
#include "../diagnostics/Diagnostics.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <chrono>

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
    float cellSize = 120.0f;
    bool useBruteForceBroadPhase = false;
    bool fineProfileEnabled = false;
    double fineBroadphaseUs = 0.0;
    double fineNarrowphaseUs = 0.0;
    double fineSolverUs = 0.0;
    double fineCorrectionUs = 0.0;
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
    using Clock = std::chrono::steady_clock;

    static double elapsedUs(Clock::time_point t0) {
        return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    }

    struct PairContact {
        RigidBody* a = nullptr;
        RigidBody* b = nullptr;
        Manifold manifold;
        bool contact = false;
        float normalImpulseAccum = 0.0f;
    };

    struct AABB {
        float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;
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

    AABB computeAABB(const RigidBody& body) const {
        AABB box;
        if (body.shape->getType() == ShapeType::Circle) {
            const CircleShape& c = static_cast<const CircleShape&>(*body.shape);
            box.minX = body.position.x - c.radius;
            box.maxX = body.position.x + c.radius;
            box.minY = body.position.y - c.radius;
            box.maxY = body.position.y + c.radius;
        } else if (body.shape->getType() == ShapeType::Plane) {
            const float huge = 1e9f;
            box.minX = -huge;
            box.minY = -huge;
            box.maxX = huge;
            box.maxY = huge;
        } else {
            const PolygonShape& p = static_cast<const PolygonShape&>(*body.shape);
            box.minX = std::numeric_limits<float>::max();
            box.minY = std::numeric_limits<float>::max();
            box.maxX = -std::numeric_limits<float>::max();
            box.maxY = -std::numeric_limits<float>::max();
            for (const Vec2& v : p.vertices) {
                Vec2 w = v.rotated(body.angle) + body.position;
                box.minX = std::min(box.minX, w.x);
                box.minY = std::min(box.minY, w.y);
                box.maxX = std::max(box.maxX, w.x);
                box.maxY = std::max(box.maxY, w.y);
            }
        }
        return box;
    }

    static int64_t pairKey(int a, int b) {
        return (static_cast<int64_t>(a) << 32) | static_cast<uint32_t>(b);
    }

    static int64_t cellKey(int cx, int cy) {
        return (static_cast<int64_t>(cx) << 32) ^ static_cast<uint32_t>(cy);
    }

    std::vector<int64_t> collectCandidates() const {
        static constexpr int CELL_RANGE_CLAMP = 1000000000;

        const size_t n = bodies.size();
        std::vector<char> isPlane(n, 0);
        std::vector<int> minCx(n), maxCx(n), minCy(n), maxCy(n);
        for (size_t i = 0; i < n; ++i) {
            if (bodies[i]->shape->getType() == ShapeType::Plane) {
                isPlane[i] = 1;
                continue;
            }
            AABB box = computeAABB(*bodies[i]);
            minCx[i] = std::max(-CELL_RANGE_CLAMP, static_cast<int>(std::floor(box.minX / cellSize)));
            maxCx[i] = std::min(CELL_RANGE_CLAMP, static_cast<int>(std::floor(box.maxX / cellSize)));
            minCy[i] = std::max(-CELL_RANGE_CLAMP, static_cast<int>(std::floor(box.minY / cellSize)));
            maxCy[i] = std::min(CELL_RANGE_CLAMP, static_cast<int>(std::floor(box.maxY / cellSize)));
        }

        std::vector<int64_t> candidates;
        candidates.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            if (!isPlane[i]) continue;
            for (size_t j = 0; j < n; ++j) {
                if (isPlane[j] || j <= i) continue;
                candidates.push_back(pairKey(static_cast<int>(i), static_cast<int>(j)));
            }
        }

        std::unordered_map<int64_t, std::vector<int>> cells;
        for (size_t i = 0; i < n; ++i) {
            if (isPlane[i]) continue;
            for (int cx = minCx[i]; cx <= maxCx[i]; ++cx) {
                for (int cy = minCy[i]; cy <= maxCy[i]; ++cy) {
                    cells[cellKey(cx, cy)].push_back(static_cast<int>(i));
                }
            }
        }

        for (size_t i = 0; i < n; ++i) {
            if (isPlane[i]) continue;
            for (int cx = minCx[i]; cx <= maxCx[i]; ++cx) {
                for (int cy = minCy[i]; cy <= maxCy[i]; ++cy) {
                    auto it = cells.find(cellKey(cx, cy));
                    if (it == cells.end()) continue;
                    for (int j : it->second) {
                        if (j <= static_cast<int>(i)) continue;
                        candidates.push_back(pairKey(static_cast<int>(i), j));
                    }
                }
            }
        }
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        return candidates;
    }

    template <typename Fn>
    void forEachPair(Fn fn, bool countIntoCore = true) {
        if (useBruteForceBroadPhase || cellSize <= 0.0f) {
            for (size_t i = 0; i < bodies.size(); ++i) {
                for (size_t j = i + 1; j < bodies.size(); ++j) {
                    auto t0 = (fineProfileEnabled && countIntoCore) ? Clock::now() : Clock::time_point();
                    fn(*bodies[i], *bodies[j]);
                    if (fineProfileEnabled && countIntoCore) fineNarrowphaseUs += elapsedUs(t0);
                }
            }
            return;
        }
        auto t0 = Clock::now();
        auto candidates = collectCandidates();
        if (fineProfileEnabled && countIntoCore) fineBroadphaseUs += elapsedUs(t0);
        for (int64_t key : candidates) {
            int a = static_cast<int>(key >> 32);
            int b = static_cast<int>(static_cast<uint32_t>(key & 0xFFFFFFFF));
            auto t1 = (fineProfileEnabled && countIntoCore) ? Clock::now() : Clock::time_point();
            fn(*bodies[a], *bodies[b]);
            if (fineProfileEnabled && countIntoCore) fineNarrowphaseUs += elapsedUs(t1);
        }
    }

    void detectAllPairs(std::vector<PairContact>& out) {
        forEachPair([this, &out](RigidBody& x, RigidBody& y) {
            PairContact pc = detectPair(x, y);
            if (pc.contact) out.push_back(pc);
        });
    }

    void resolveCollisions() {
        if (fineProfileEnabled) {
            fineBroadphaseUs = fineNarrowphaseUs = fineSolverUs = fineCorrectionUs = 0.0;
        }

        std::vector<PairContact> state;
        detectAllPairs(state);

        auto sol0 = Clock::now();
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
        if (fineProfileEnabled) fineSolverUs += elapsedUs(sol0);

        auto cor0 = Clock::now();
        for (int p = 0; p < 3; ++p) {
            forEachPair([this](RigidBody& x, RigidBody& y) {
                PairContact pc = detectPair(x, y);
                if (pc.contact) {
                    applyPositionalCorrection(*pc.a, *pc.b, pc.manifold,
                                              positionCorrectionPercent, positionSlop);
                }
            }, false);
        }
        if (fineProfileEnabled) fineCorrectionUs += elapsedUs(cor0);
    }
};


