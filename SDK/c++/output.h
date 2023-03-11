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

inline void output_frameID(int frameID){
    printf("%d\n", frameID);
//    puts("OK");
    fflush(stdout);
}

inline void forward(int robot_id, double velocity) {
    printf("forward %d %lf\n", robot_id, velocity);
    fflush(stdout);
}

inline void backward(int robot_id, double velocity) {
    printf("forward %d %lf\n", robot_id, -velocity);
    fflush(stdout);
}

inline void rotate(int robot_id, double omega) {
    printf("rotate %d %lf\n", robot_id, omega);
    fflush(stdout);
}

inline void buy(int robot_id) {
    printf("buy %d\n", robot_id);
    fflush(stdout);
}

inline void sell(int robot_id) {
    printf("sell %d\n", robot_id);
    fflush(stdout);
}

//inline void destory(int robot_id) {
//
//}

#endif //CODECRAFTSDK_OUTPUT_H
