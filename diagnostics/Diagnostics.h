#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <cstdio>

struct PhaseTiming {
    const char* name;
    double microseconds;
};

class FrameProfiler {
public:
    void reset() {
        phases.clear();
        totalMicroseconds = 0.0;
    }

    void beginPhase(const char* name) {
        currentName = name;
        phaseStart = Clock::now();
    }

    void endPhase() {
        auto end = Clock::now();
        double us = std::chrono::duration<double, std::micro>(end - phaseStart).count();
        phases.push_back({ currentName, us });
        totalMicroseconds += us;
    }

    const std::vector<PhaseTiming>& getPhases() const { return phases; }
    double getTotalMicroseconds() const { return totalMicroseconds; }

private:
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point phaseStart;
    const char* currentName = "";
    std::vector<PhaseTiming> phases;
    double totalMicroseconds = 0.0;
};

struct InstabilityEvent {
    enum Type { NaN_detected, Inf_detected, velocity_threshold, position_threshold };
    Type type;
    int bodyIndex;
    float value;
};

class InstabilityDetector {
public:
    float maxVelocity = 1e6f;
    float maxPosition = 1e9f;

    void reset() { events.clear(); }

    void checkBody(int index, float x, float y, float vx, float vy) {
        if (std::isnan(x) || std::isnan(y)) {
            events.push_back({ InstabilityEvent::NaN_detected, index, std::isnan(x) ? x : y });
        }
        if (std::isinf(x) || std::isinf(y)) {
            events.push_back({ InstabilityEvent::Inf_detected, index, std::isinf(x) ? x : y });
        }
        if (std::isnan(vx) || std::isnan(vy)) {
            events.push_back({ InstabilityEvent::NaN_detected, index, std::isnan(vx) ? vx : vy });
        }
        if (std::isinf(vx) || std::isinf(vy)) {
            events.push_back({ InstabilityEvent::Inf_detected, index, std::isinf(vx) ? vx : vy });
        }
        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed > maxVelocity) {
            events.push_back({ InstabilityEvent::velocity_threshold, index, speed });
        }
        if (std::fabs(x) > maxPosition || std::fabs(y) > maxPosition) {
            events.push_back({ InstabilityEvent::position_threshold, index,
                               std::fabs(x) > std::fabs(y) ? x : y });
        }
    }

    bool hasEvents() const { return !events.empty(); }
    const std::vector<InstabilityEvent>& getEvents() const { return events; }

    std::string describeEvent(const InstabilityEvent& e) const {
        char buf[128];
        switch (e.type) {
        case InstabilityEvent::NaN_detected:
            std::snprintf(buf, sizeof(buf), "Body %d: NaN detected (value=%.6f)", e.bodyIndex, e.value);
            break;
        case InstabilityEvent::Inf_detected:
            std::snprintf(buf, sizeof(buf), "Body %d: Inf detected (value=%.6f)", e.bodyIndex, e.value);
            break;
        case InstabilityEvent::velocity_threshold:
            std::snprintf(buf, sizeof(buf), "Body %d: velocity %.1f exceeds threshold", e.bodyIndex, e.value);
            break;
        case InstabilityEvent::position_threshold:
            std::snprintf(buf, sizeof(buf), "Body %d: position %.1f exceeds threshold", e.bodyIndex, e.value);
            break;
        }
        return std::string(buf);
    }

private:
    std::vector<InstabilityEvent> events;
};
