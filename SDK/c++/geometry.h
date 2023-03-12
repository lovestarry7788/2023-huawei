#ifndef HW2023_GEOMETRY_H
#define HW2023_GEOMETRY_H

#include <cmath>

namespace Geometry {
    constexpr double pi = acos(-1);
    constexpr double eps = 1e-10;
    int dcmp(double x) { if (fabs(x) < eps) return 0; return x < 0 ? -1 : 1; }

    struct Point { double x, y;};
    using Vector = Point;

    Vector operator+(const Vector& A, const Vector& B) { return Vector{A.x + B.x, A.y + B.y}; }
    Vector operator-(const Point& A, const Point& B) { return Vector{A.x - B.x, A.y - B.y}; }
    Vector operator*(const Vector& A, double p) { return Vector{A.x * p, A.y * p}; }
    Vector operator/(const Vector& A, double p) { return Vector{A.x / p, A.y / p}; }
    bool operator==(const Point& a, const Point& b) { return a.x == b.x && a.y == b.y; }
    bool operator<(const Point& p1, const Point& p2) {
        if (p1.x != p2.x) return p1.x < p2.x;
        return p1.y < p2.y;
    }

    // 两点间距离
    double Dist(double x1, double y1, double x2, double y2);

    // 匀变速运动距离/角度
    double UniformVariableDist(double a, double v, double aim_v);

    double Dist(double x1, double y1, double x2, double y2) {
        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }

    double UniformVariableDist(double a, double v, double aim_v) {
        return abs(v * v - aim_v * aim_v) / (2 * a);
    }

    double Dot(const Vector& A, const Vector& B) { return A.x * B.x + A.y * B.y; }

    double Length(const Vector& A) { return sqrt(Dot(A, A)); }

    double Angle(const Vector& A, const Vector& B) { return acos(Dot(A, B) / Length(A) / Length(B)); }

}

#endif