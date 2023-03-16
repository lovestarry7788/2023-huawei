#include "geometry.h"

//using namespace Geometry;

int Geometry::dcmp(double x) { if (fabs(x) < eps) return 0; return x < 0 ? -1 : 1; }
Geometry::Point Geometry::operator+(const Geometry::Point& A, const Geometry::Point& B) { return Geometry::Point{A.x + B.x, A.y + B.y}; }
Geometry::Point Geometry::operator-(const Geometry::Point& A, const Geometry::Point& B) { return Geometry::Point{A.x - B.x, A.y - B.y}; }
Geometry::Point Geometry::operator*(const Geometry::Point& A, double p) { return Geometry::Point{A.x * p, A.y * p}; }
Geometry::Point Geometry::operator/(const Geometry::Point& A, double p) { return Geometry::Point{A.x / p, A.y / p}; }
bool Geometry::operator==(const Geometry::Point& a, const Geometry::Point& b) { return a.x == b.x && a.y == b.y; }
bool Geometry::operator<(const Geometry::Point& p1, const Geometry::Point& p2) {
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

double Geometry::Dot(const Geometry::Point& A, const Geometry::Point& B) { return A.x * B.x + A.y * B.y; }

double Geometry::Length(const Geometry::Point& A) { return sqrt(Dot(A, A)); }

double Geometry::InterAngle(const Geometry::Point& A, const Geometry::Point& B) { return acos(Dot(A, B) / Length(A) / Length(B)); }

double Geometry::MinRadius(double dist, double theta) {
    return dist / (2 * std::max(1e-8, sin(theta)));
}

double Geometry::AngleReg(double r) {if (r > PI) r -= 2*PI; else if (r < -PI) r += 2*PI; return r;}

// Geometry::Angle::Angle(double r) { this->r = reg(r); }
// double Geometry::Angle::reg(double r) {if (r > PI) r -= 2*PI; else if (r < -PI) r += 2*PI; return r;}
// Angle Geometry::operator+(const Angle& A, const Angle& B) { Angle r{Geometry::Angle::reg(A.r + B.r)}; return r; }
// Angle Geometry::operator-(const Angle& A, const Angle& B) { Angle r{Geometry::Angle::reg(A.r - B.r)}; return r; }
