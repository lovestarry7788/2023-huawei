#ifndef HW2023_GEOMETRY_H
#define HW2023_GEOMETRY_H

#include <cmath>

// double UniformVariableDist(double a, double v, double aim_v);

namespace Geometry {
    constexpr double pi = acos(-1);
    constexpr double eps = 1e-10;

    struct Point { double x, y;};
    using Vector = Point;

    int dcmp(double x) ;
    Vector operator+(const Vector& A, const Vector& B);
    Vector operator-(const Point& A, const Point& B);
    Vector operator*(const Vector& A, double p);
    Vector operator/(const Vector& A, double p);
    bool operator==(const Point& a, const Point& b);
    bool operator<(const Point& p1, const Point& p2);

    // 两点间距离
    double Dist(double x1, double y1, double x2, double y2);

    // 匀变速运动距离/角度
    double UniformVariableDist(double a, double v, double aim_v);

    // 圆周运动最小半径。
    double MinRadius(double dist, double theta);

    double Dot(const Vector& A, const Vector& B) ;

    double Length(const Vector& A);

    double Angle(const Vector& A, const Vector& B);
}

#endif