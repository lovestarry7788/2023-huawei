//
// Created by gllonkxc on 2023/3/11.
//

#ifndef CODECRAFTSDK_OUTPUT_H
#define CODECRAFTSDK_OUTPUT_H

#include <iostream>

inline void outputOK() {
    puts("OK");
    fflush(stdout);
}

inline void outputOK(int frameID){
    printf("%d\n", frameID);
    puts("OK");
    fflush(stdout);
}

#endif //CODECRAFTSDK_OUTPUT_H
