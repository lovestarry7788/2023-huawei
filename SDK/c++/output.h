#ifndef HW2023_OUTPUT_H
#define HW2023_OUTPUT_H

#include <iostream>
#include <vector>


namespace Output {
    extern std::vector<std::string> Operation; // string_view会乱码，故换成string --WY

    void Forward(int robot_id, double velocity);
    void Rotate(int robot_id, double radius);

    void Buy(int robot_id);

    void Sell(int robot_id);

    void Destroy(int robot_id);
    void Print(int frame_id);
};


#endif