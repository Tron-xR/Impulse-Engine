#pragma once

#include <cmath>
#include <limits>

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG2RAD = PI / 180.0f;
constexpr float EPSILON = std::numeric_limits<float>::epsilon();

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    constexpr Vec2(float x, float y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& other) const { return { x + other.x, y + other.y }; }
    Vec2 operator-(const Vec2& other) const { return { x - other.x, y - other.y }; }
    Vec2 operator*(float scalar) const { return { x * scalar, y * scalar }; }
    Vec2 operator/(float scalar) const { return { x / scalar, y / scalar }; }
    Vec2 operator-() const { return { -x, -y }; }

    Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
    Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }
    Vec2& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }

    bool operator==(const Vec2& other) const {
        return std::fabs(x - other.x) <= EPSILON && std::fabs(y - other.y) <= EPSILON;
    }

    float lengthSq() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSq()); }

    Vec2 normalized() const {
        float len = length();
        if (len < EPSILON) return { 0.0f, 0.0f };
        return { x / len, y / len };
    }

    float dot(const Vec2& other) const { return x * other.x + y * other.y; }

    float cross(const Vec2& other) const { return x * other.y - y * other.x; }

    Vec2 perpendicular() const { return { -y, x }; }

    Vec2 rotated(float radians) const {
        float c = std::cos(radians);
        float s = std::sin(radians);
        return { x * c - y * s, x * s + y * c };
    }
};

inline Vec2 operator*(float scalar, const Vec2& v) { return v * scalar; }

inline float dot(const Vec2& a, const Vec2& b) { return a.dot(b); }

inline float cross(const Vec2& a, const Vec2& b) { return a.cross(b); }

inline Vec2 normalize(const Vec2& v) { return v.normalized(); }

inline Vec2 rotate(const Vec2& v, float radians) { return v.rotated(radians); }
