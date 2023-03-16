//
// Created by gllonkxc on 2023/3/16.
//

#include "Environment.h"

Environment::Environment()
{
    //barrier初始化
    for (float i = 0; i < 50; i += gride_size) {
        barrier.push_back(Point(0, i));
        barrier.push_back(Point(49.5, i + gride_size));
        barrier.push_back(Point(i + gride_size, 0));
        barrier.push_back(Point(i, 49.5));
    }

}