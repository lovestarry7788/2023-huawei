#include "robot.h"
#include <iostream>
#include <algorithm>

using namespace std;

bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}

char map_[101][101];

int main() {
    for(int i = 0; i < 100; ++i) {
        for(int j = 0; j < 100; ++j) {
            scanf("\n%c",&map_[i][j]);
        }
    }
    readUntilOK();

    shared_ptr<Robot> robot[4];
    int frameID, coins;
    while (scanf("%d%d", &frameID, &coins) != EOF) {
        int K;
        scanf("%d",&K);
        for(int i = 1; i <= K; ++i) {

        }
        for(size_t id = 0; id < 4; ++id) {
            size_t workbench, carry_id;
            double time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0;
            scanf("%zd%zd%lf%lf%lf%lf%lf%lf%lf",&workbench, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity,
                  &orient, &x0, &y0);
            robot[id] = new Robot{id, workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0};
        }

        readUntilOK();
        fflush(stdout);
    }
    return 0;
}
