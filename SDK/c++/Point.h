//
// Created by gllonkxc on 2023/3/16.
//

#ifndef INC_2023_HUAWEI_POINT_H
#define INC_2023_HUAWEI_POINT_H

class Point{
public:
    double x, y;
    Point();
    Point(double x0_, double y0_); //{ x0_ = x; y0_ = y;}
    Point(const Point &b);// { x0_ = b.x0_; y0_ = b.y0_; }
};

#endif //INC_2023_HUAWEI_POINT_H
