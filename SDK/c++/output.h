//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_OUTPUT_H
#define HW2023_OUTPUT_H

#include <vector>
#include <string_view>

class Output {
private:
    int frame_id_;
    std::vector<std::string_view> Operation;

public:
    void SetFrame(int frame_id);
    void Forward(int robot_id, double velocity);
    void Rotate(int robot_id, double radius);
    void Buy(int robot_id);
    void Sell(int robot_id);
    void Destroy(int robot_id);
    void Print();
};

#endif //HW2023_OUTPUT_H
