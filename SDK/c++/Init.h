//
// Created by gllonkxc on 2023/3/10.
//
#ifndef HW2023_INIT_H
#define HW2023_INIT_H

#pragma once
#include <iostream>
#include <cmath>
#include <memory>
using namespace std;

inline bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
    }
    return false;
}

#endif
