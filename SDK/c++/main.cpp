#include "robot.h"
#include "workbench.h"
#include <iostream>
#include <vector>
#include <string_view>
#include <fstream>

namespace Input {
    int frameID, coins, K;
    char map_[101][101];
    std::vector<std::shared_ptr<Workbench> > workbench;
    std::vector<std::shared_ptr<Robot> > robot;

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

    void ScanMap() {
        // 地图输入开始
        for(int i = 0; i < 100; ++i) {
            for(int j = 0; j < 100; ++j) {
                scanf("\n%c",&map_[i][j]);
            }
        }
        readUntilOK();
        puts("OK");
        fflush(stdout);
    }

    bool ScanFrame() {
        while (scanf("%d%d", &frameID, &coins) != EOF) {
            scanf("%d",&K);
            workbench.resize(K);
            for(int i = 0; i < K; ++i) {
                int type_id, frame_remain, materials_status, product_status;
                double x0, y0;
                scanf("%d%lf%lf%d%d%d",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
                workbench[i] = std::make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
            }

            robot.resize(4);
            for(int id = 0; id < 4; ++id) {
                int workbench_id, carry_id;
                double time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0;
                scanf("%d%d%lf%lf%lf%lf%lf%lf%lf",&workbench_id, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity,
                      &orient, &x0, &y0);
                robot[id] = std::make_shared<Robot>(id, workbench_id, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0);
            }

            readUntilOK();
            return true;
        }
        return false;
    }
};

namespace Output {
    std::vector<std::string> Operation; // string_view会乱码，故换成string --WY

    void Forward(int robot_id, double velocity) {
        Operation.emplace_back(std::string{"forward " + std::to_string(robot_id) + " " + std::to_string(velocity)});
    }

    void Rotate(int robot_id, double radius) {
        Operation.emplace_back(std::string{"rotate " + std::to_string(robot_id) + " " + std::to_string(radius)});
    }

    void Buy(int robot_id) {
        Operation.emplace_back(std::string{"buy " + std::to_string(robot_id)});
    }

    void Sell(int robot_id) {
        Operation.emplace_back(std::string{"sell " + std::to_string(robot_id)});
    }

    void Destroy(int robot_id) {
        Operation.emplace_back(std::string{"destory " + std::to_string(robot_id)});
    }

    void Print(int frame_id) {
        printf("%d\n", frame_id);
        for(const auto& u: Operation) {
            printf("%s", u.data());
        }
        puts("OK\n");
        fflush(stdout);
        Operation.clear();
    }
};

struct DebugLog {
    std::ofstream ofs;
    DebugLog(): ofs("main.log") {}
    template<class T, class... A> void print(T&& x, A&&... a) { 
        ofs<<x; (int[]){(ofs<< ' '<< a,0)...}; ofs<<'\n'; 
    }
} Log;

namespace Solution1 {
    using namespace Input;
    using namespace Output;
    void Solve() {
        Input::ScanMap();
        while(Input::ScanFrame()) {
            // Solution
            double forward, rotate;
            robot[0]->ToPoint(10, 10, forward, rotate);

            Output::Forward(0, forward);
            Output::Rotate(0, rotate);

            Log.print(Input::frameID);
            for (auto i : Output::Operation)
                Log.print(i);
            // Log.print(forward, rotate);

            Output::Print(Input::frameID);
        }
    }
}

int main() {
    Solution1::Solve();
    return 0;
}
