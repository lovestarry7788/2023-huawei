#include "geometry.h"

using namespace Geometry;

int Geometry::dcmp(double x) { if (fabs(x) < eps) return 0; return x < 0 ? -1 : 1; }
Vector Geometry::operator+(const Vector& A, const Vector& B) { return Vector{A.x + B.x, A.y + B.y}; }
Vector Geometry::operator-(const Point& A, const Point& B) { return Vector{A.x - B.x, A.y - B.y}; }
Vector Geometry::operator*(const Vector& A, double p) { return Vector{A.x * p, A.y * p}; }
Vector Geometry::operator/(const Vector& A, double p) { return Vector{A.x / p, A.y / p}; }
bool Geometry::operator==(const Point& a, const Point& b) { return a.x == b.x && a.y == b.y; }
bool Geometry::operator<(const Point& p1, const Point& p2) {
    if (p1.x != p2.x) return p1.x < p2.x;
    return p1.y < p2.y;
}

double Geometry::Dist(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// 匀变速运动，估计开始减速距离
double Geometry::UniformVariableDist(double a, double v, double aim_v) {
    return abs(v * v - aim_v * aim_v) / (2 * a);
}

// v -> v_max -> v_max -> 0，求时间
double Geometry::UniformVariableDist2(double a, double x, double v, double v_max) {
    double t = (2 * v_max - v) / a;
    x -= UniformVariableDist(a, v, v_max);
    x -= UniformVariableDist(a, v_max, 0);
    t += x / v_max;
    return t;
}

double Geometry::Dot(const Vector& A, const Vector& B) { return A.x * B.x + A.y * B.y; }

double Geometry::Length(const Vector& A) { return sqrt(Dot(A, A)); }

double Geometry::InterAngle(const Vector& A, const Vector& B) { return acos(Dot(A, B) / Length(A) / Length(B)); }

double Geometry::MinRadius(double dist, double theta) {
    return dist / (2 * std::max(1e-8, sin(theta)));
}

double Geometry::AngleReg(double r) {if (r > PI) r -= 2*PI; else if (r < -PI) r += 2*PI; return r;}

Point Geometry::GetLineIntersection2(Point P, Vector v, Point Q, Vector w) {
    Vector u = P - Q;
    double t = Cross(w, u) / Cross(v, w);
    return P + v * t;
}

double Geometry::Cross(const Vector& A, const Vector& B) { return A.x * B.y - A.y * B.x; }


// Geometry::Angle::Angle(double r) { this->r = reg(r); }
// double Geometry::Angle::reg(double r) {if (r > PI) r -= 2*PI; else if (r < -PI) r += 2*PI; return r;}
// Angle Geometry::operator+(const Angle& A, const Angle& B) { Angle r{Geometry::Angle::reg(A.r + B.r)}; return r; }
// Angle Geometry::operator-(const Angle& A, const Angle& B) { Angle r{Geometry::Angle::reg(A.r - B.r)}; return r; }
