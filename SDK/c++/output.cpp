//
// Created by 刘智杰 on 2023/3/10.
//
#include "output.h"
#include <vector>
#include <string_view>
#include <string>


void Output::SetFrame(int frame_id) {
    frame_id_ = frame_id;
}

void Output::Forward(int robot_id, double velocity) {
    Operation.emplace_back(std::string_view{"forward " + std::to_string(robot_id) + " " + std::to_string(velocity)});
}

void Output::Rotate(int robot_id, double radius) {
    Operation.emplace_back(std::string_view{"rotate " + std::to_string(robot_id) + " " + std::to_string(radius)});
}

void Output::Buy(int robot_id) {
    Operation.emplace_back(std::string_view{"buy " + std::to_string(robot_id)});
}

void Output::Sell(int robot_id) {
    Operation.emplace_back(std::string_view{"sell " + std::to_string(robot_id)});
}

void Output::Destroy(int robot_id) {
    Operation.emplace_back(std::string_view{"destory " + std::to_string(robot_id)});
}

void Output::Print() {
    printf("%d\n", frame_id_);
    for(const auto& u: Operation) {
        printf("%s", u.data());
    }
    puts("OK\n");
    fflush(stdout);
}