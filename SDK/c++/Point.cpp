//
// Created by gllonkxc on 2023/3/16.
//

#include "Point.h"

Point operator+(const Point& A, const Point& B) { return Point{A.x + B.x, A.y + B.y}; }
Point operator-(const Point& A, const Point& B) { return Point{A.x - B.x, A.y - B.y}; }
Point operator*(const Point& A, double p) { return Point{A.x * p, A.y * p}; }
Point operator/(const Point& A, double p) { return Point{A.x / p, A.y / p}; }
bool operator==(const Point& a, const Point& b) { return a.x == b.x && a.y == b.y; }
bool operator<(const Point& p1, const Point& p2) {
    if (p1.x != p2.x) return p1.x < p2.x;
    return p1.y < p2.y;
}

Point::Point() {x = 0; y = 0;}
Point::Point(double x, double y) { x = x; y = y;}
Point::Point(const Point &b) { x = b.x; y = b.y; }
