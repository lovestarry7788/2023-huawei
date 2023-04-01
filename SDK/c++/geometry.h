#ifndef HW2023_GEOMETRY_H
#define HW2023_GEOMETRY_H

#include <cmath>
#include <algorithm>
#include <iostream>

// double UniformVariableDist(double a, double v, double aim_v);

namespace Geometry {
    static constexpr double PI = acos(-1);
    static constexpr double eps = 1e-10;

    struct Point{
        double x, y;
        Point(double x_ = 0, double y_ = 0) : x(x_), y(y_){}
    };
    using Vector = Point;

    std::ostream& operator<<(std::ostream& os, const Point& p);

    int dcmp(double x) ;
    Vector operator+(const Vector& A, const Vector& B);
    Vector operator-(const Point& A, const Point& B);
    Vector operator*(const Vector& A, double p);
    Vector operator/(const Vector& A, double p);
    bool operator==(const Point& a, const Point& b);
    bool operator<(const Point& p1, const Point& p2);

    Vector Rotate(Vector A, double rad);
    
    // 两点间距离
    double Dist(double x1, double y1, double x2, double y2);
    double Dist(Point a, Point b);

    // 匀变速运动距离/角度
    double UniformVariableDist(double a, double v, double aim_v);

    // v -> v_max -> v_max -> 0，求时间
    double UniformVariableDist2(double a, double x, double v, double v_max);

    double MinRadius2(double x, double y, double theta);

    // 圆周运动最小半径。
    double MinRadius(double dist, double theta);

    double Dot(const Vector& A, const Vector& B) ;

    double Cross(const Vector& A, const Vector& B) ;

    double Length(const Vector& A);

    // 两向量夹角
    double InterAngle(const Vector& A, const Vector& B);

    double AngleReg(double r);

    Point GetLineIntersection2(Point P, Vector v, Point Q, Vector w);

    double DistanceToSegment(const Point& P, const Point& A, const Point& B);

    double Angle(const Vector& a);

    bool CheckCross(const Vector& A, const Vector& B, std::vector<Geometry::Point> v);

    // struct Angle {
    //     double r; // [-pi, pi]
    //     Angle(double r = 0);
    //     static double reg(double r);
    // };
    // Angle operator+(const Angle& A, const Angle& B); // 不using可以使用吗？
    // Angle operator-(const Angle& A, const Angle& B);
}

#endif