//
// Created by gllonkxc on 2023/3/16.
//

#ifndef CODECRAFTSDK_ENVIRONMENT_H
#define CODECRAFTSDK_ENVIRONMENT_H

#include "Point.h"
//#include "Environment.h"
#include <vector>

class Environment {
public:
    Environment();

    std::vector<Point> barrier;
    float gride_size = 0.5;

};

#endif //CODECRAFTSDK_ENVIRONMENT_H
