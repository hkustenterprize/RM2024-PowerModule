#pragma once

#include "Math.hpp"
namespace SGChassis
{
namespace SGUtil
{
template <typename T>
struct Quadruple
{
    T x, y, z, w;
    Quadruple() : x(0), y(0), z(0), w(0) {}
    Quadruple(T x_, T y_, T z_, T w_) : x(x_), y(y_), z(z_), w(w_) {}
    Quadruple(const Quadruple<T> &other) : x(other.x), y(other.y), z(other.z), w(other.w) {}
    // Quadruple(const Quadruple<const T> &other) : x(other.x), y(other.y), z(other.z), w(other.w) {}
    Quadruple<T> &operator=(const Quadruple<T> &other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
        return *this;
    }

    T &operator[](uint8_t index) { return (index == 0) ? x : (index == 1) ? y : (index == 2) ? z : w; }
    const T &operator[](uint8_t index) const { return (index == 0) ? x : (index == 1) ? y : (index == 2) ? z : w; }
};

template <typename T>
struct Triple
{
    T x, y, z;
    Triple() : x(0), y(0), z(0) {}
    Triple(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
    Triple(const Triple<T> &other) : x(other.x), y(other.y), z(other.z) {}
    Triple<T> &operator=(const Triple<T> &other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    T *operator[](uint8_t index) { return (index == 0) ? &x : (index == 1) ? &y : &z; }
    const T *operator[](uint8_t index) const { return (index == 0) ? &x : (index == 1) ? &y : &z; }
};

constexpr static float RPM_TO_RAD = 0.10471975511965977f;

inline float rpm2rad(float rpm) { return rpm * SGUtil::RPM_TO_RAD; }

inline float rad2rpm(float rad) { return rad / SGUtil::RPM_TO_RAD; }

inline float floatEqual(float a, float b) { return fabs(a - b) < 1e-6f; }

inline float floatEqual(float a, float b, float c) { return floatEqual(a, b) && floatEqual(b, c); }

inline float clampSteerError(float value)
{
    return value > (float)M_PI ? value - 2 * (float)M_PI : value < -(float)M_PI ? value + 2 * (float)M_PI : value;
}

}  // namespace SGUtil
}  // namespace SGChassis