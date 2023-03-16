#include "output.h"
#include "log.h"

std::vector<std::string> Output::Operation; // string_view会乱码，故换成string --WY

void Output::Forward(int robot_id, double velocity) {
    Operation.emplace_back(std::string{"forward " + std::to_string(robot_id) + " " + std::to_string(velocity)});
}

void Output::Rotate(int robot_id, double radius) {
    Operation.emplace_back(std::string{"rotate " + std::to_string(robot_id) + " " + std::to_string(radius)});
}

void Output::Buy(int robot_id) {
    Operation.emplace_back(std::string{"buy " + std::to_string(robot_id)});
}

void Output::Sell(int robot_id) {
    Operation.emplace_back(std::string{"sell " + std::to_string(robot_id)});
}

void Output::Destroy(int robot_id) {
    Operation.emplace_back(std::string{"destroy " + std::to_string(robot_id)});
}

void Output::Print(int frame_id) {
    printf("%d\n", frame_id);
//    Log::print(frame_id);
    for(const auto& u: Operation) {
        printf("%s\n", u.data()); // 记得换行
//        Log::print(u.data());
    }
    puts("OK\n");
    fflush(stdout);
    Operation.clear();
}
