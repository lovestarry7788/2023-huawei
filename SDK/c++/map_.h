//
// Created by gllonkxc on 2023/3/11.
//

#ifndef CODECRAFTSDK_MAP__H
#define CODECRAFTSDK_MAP__H
#include "Init.h"
#include <cstdio>

class map_{
//private:
public:
    char map_[105][105];
    inline void init() {
        for(int i = 0; i < 100; ++i) {
            for(int j = 0; j < 100; ++j) {
                scanf("\n%c",&map_[i][j]);
            }
        }
        readUntilOK();
        outputOK();
    }
};

#endif //CODECRAFTSDK_MAP__H
