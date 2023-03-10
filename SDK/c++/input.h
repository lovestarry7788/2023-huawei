//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_INPUT_H
#define HW2023_INPUT_H

#include "robot.h"
#include "workbench.h"

class Input {
public:
    bool readUntilOK();
    void ScanMap(char **map_);
    void ScanFrame(int* frameID, int* coins, int *K, std::vector<std::shared_ptr<Workbench> > workbench, std::vector<std::shared_ptr<Robot> > robot);
};

#endif // HW2023_INPUT_H
