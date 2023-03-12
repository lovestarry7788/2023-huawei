#include "geometry.h"

using namespace Geometry;

// double UniformVariableDist(double a, double v, double aim_v) {
//     return abs(v * v - aim_v * aim_v) / (2 * a);
// }



int Geometry::dcmp(double x) { if (fabs(x) < eps) return 0; return x < 0 ? -1 : 1; }
Vector operator+(const Vector& A, const Vector& B) { return Vector{A.x + B.x, A.y + B.y}; }
Vector operator-(const Point& A, const Point& B) { return Vector{A.x - B.x, A.y - B.y}; }
Vector operator*(const Vector& A, double p) { return Vector{A.x * p, A.y * p}; }
Vector operator/(const Vector& A, double p) { return Vector{A.x / p, A.y / p}; }
bool operator==(const Point& a, const Point& b) { return a.x == b.x && a.y == b.y; }
bool operator<(const Point& p1, const Point& p2) {
    if (p1.x != p2.x) return p1.x < p2.x;
    return p1.y < p2.y;
}

double Geometry::Dist(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

double Geometry::UniformVariableDist(double a, double v, double aim_v) {
    return abs(v * v - aim_v * aim_v) / (2 * a);
}

double Geometry::Dot(const Vector& A, const Vector& B) { return A.x * B.x + A.y * B.y; }

double Geometry::Length(const Vector& A) { return sqrt(Dot(A, A)); }

double Geometry::Angle(const Vector& A, const Vector& B) { return acos(Dot(A, B) / Length(A) / Length(B)); }