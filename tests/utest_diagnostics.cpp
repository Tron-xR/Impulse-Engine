#include <cmath>
#include <iostream>
#include <cstdlib>

#include "diagnostics/Diagnostics.h"

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " #cond " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return 1; \
        } \
    } while(0)

int testProfilerBasic() {
    FrameProfiler p;
    p.beginPhase("test");
    volatile float x = 0;
    for (int i = 0; i < 1000; ++i) x += 1.0f;
    p.endPhase();
    CHECK(p.getPhases().size() == 1);
    CHECK(p.getPhases()[0].microseconds > 0.0);
    CHECK(p.getTotalMicroseconds() > 0.0);
    std::cout << "  PASS: Profiler basic (phase=" << p.getPhases()[0].microseconds << " us)\n";
    return 0;
}

int testProfilerMultiplePhases() {
    FrameProfiler p;
    p.beginPhase("A");
    p.endPhase();
    p.beginPhase("B");
    p.endPhase();
    p.beginPhase("C");
    p.endPhase();
    CHECK(p.getPhases().size() == 3);
    CHECK(p.getPhases()[0].name[0] == 'A');
    CHECK(p.getPhases()[1].name[0] == 'B');
    CHECK(p.getPhases()[2].name[0] == 'C');
    std::cout << "  PASS: Profiler multiple phases (total=" << p.getTotalMicroseconds() << " us)\n";
    return 0;
}

int testDetectorNaN() {
    InstabilityDetector d;
    d.checkBody(0, std::nanf(""), 0.0f, 0.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents().size() == 1);
    CHECK(d.getEvents()[0].type == InstabilityEvent::NaN_detected);
    std::cout << "  PASS: Detector catches NaN\n";
    return 0;
}

int testDetectorInf() {
    InstabilityDetector d;
    d.checkBody(1, 0.0f, std::numeric_limits<float>::infinity(), 0.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents()[0].type == InstabilityEvent::Inf_detected);
    std::cout << "  PASS: Detector catches Inf\n";
    return 0;
}

int testDetectorVelocityThreshold() {
    InstabilityDetector d;
    d.maxVelocity = 100.0f;
    d.checkBody(2, 0.0f, 0.0f, 500.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents()[0].type == InstabilityEvent::velocity_threshold);
    std::cout << "  PASS: Detector catches velocity threshold\n";
    return 0;
}

int testDetectorPositionThreshold() {
    InstabilityDetector d;
    d.maxPosition = 500.0f;
    d.checkBody(3, 1000.0f, 0.0f, 0.0f, 0.0f);
    CHECK(d.hasEvents());
    CHECK(d.getEvents()[0].type == InstabilityEvent::position_threshold);
    std::cout << "  PASS: Detector catches position threshold\n";
    return 0;
}

int testDetectorCleanBody() {
    InstabilityDetector d;
    d.checkBody(4, 100.0f, -200.0f, 50.0f, -30.0f);
    CHECK(!d.hasEvents());
    std::cout << "  PASS: Detector clean body (no false positives)\n";
    return 0;
}

int testDetectorReset() {
    InstabilityDetector d;
    d.checkBody(0, std::nanf(""), 0.0f, 0.0f, 0.0f);
    CHECK(d.hasEvents());
    d.reset();
    CHECK(!d.hasEvents());
    std::cout << "  PASS: Detector reset clears events\n";
    return 0;
}

int testProfilerReset() {
    FrameProfiler p;
    p.beginPhase("A");
    p.endPhase();
    CHECK(p.getPhases().size() == 1);
    p.reset();
    CHECK(p.getPhases().size() == 0);
    std::cout << "  PASS: Profiler reset clears phases\n";
    return 0;
}

int testDetectorDescribe() {
    InstabilityDetector d;
    d.checkBody(5, std::nanf(""), 0.0f, 0.0f, 0.0f);
    std::string desc = d.describeEvent(d.getEvents()[0]);
    CHECK(desc.find("Body 5") != std::string::npos);
    CHECK(desc.find("NaN") != std::string::npos);
    std::cout << "  PASS: Detector describe produces readable string\n";
    return 0;
}

int main() {
    std::cout << "Diagnostics Unit Tests\n";
    std::cout << "======================\n";

    int failures = 0;
    failures += testProfilerBasic();
    failures += testProfilerMultiplePhases();
    failures += testDetectorNaN();
    failures += testDetectorInf();
    failures += testDetectorVelocityThreshold();
    failures += testDetectorPositionThreshold();
    failures += testDetectorCleanBody();
    failures += testDetectorReset();
    failures += testProfilerReset();
    failures += testDetectorDescribe();

    std::cout << "======================\n";
    if (failures == 0) {
        std::cout << "All tests passed.\n";
    } else {
        std::cout << failures << " test(s) FAILED.\n";
    }
    return failures;
}
