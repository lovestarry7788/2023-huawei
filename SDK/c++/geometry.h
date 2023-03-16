#ifndef HW2023_GEOMETRY_H
#define HW2023_GEOMETRY_H

#include <cmath>
#include <algorithm>

// double UniformVariableDist(double a, double v, double aim_v);

namespace Geometry {
    const double PI = acos(-1);
    static constexpr double eps = 1e-10;

    struct Point{
        double x, y;
        Point(double x_, double y_) : x(x_), y(y_){}
    };
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

    // 两向量夹角
    double InterAngle(const Vector& A, const Vector& B);

    double AngleReg(double r);
    // struct Angle {
    //     double r; // [-pi, pi]
    //     Angle(double r = 0);
    //     static double reg(double r);
    // };
    // Angle operator+(const Angle& A, const Angle& B); // 不using可以使用吗？
    // Angle operator-(const Angle& A, const Angle& B);
}

#endif