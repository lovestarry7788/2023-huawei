#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <fstream>

using namespace std;

int main() {
    // std::ofstream ofs("result.txt");

    FILE *fp2 = fopen("result.txt", "w+");
    if(fp2==nullptr) {
        exit(0);
    }
    fprintf(fp2, "\n");
    fflush(fp2);

    for(double sever_one = 1.8; sever_one <= 2.15; sever_one += 0.1) {
        for(double four_five_six_one = 1.3; four_five_six_one <= 1.75; four_five_six_one += 0.1) {
            for(double sever_two = 1.0; sever_two <= 1.45; sever_two += 0.1) {
                for(double four_five_six_two = 1.0; four_five_six_two <= 1.45; four_five_six_two += 0.1) {
                    FILE *fp = fopen("config.txt", "w+");
                    fprintf(fp, "%f %f %f %f %f\n", sever_one, four_five_six_one, sever_two, four_five_six_two, 1.0);
                    printf("Calculate! sever_one: %f, four_five_six_one: %f, sever_two: %f, four_five_six_two: %f, sever_three: %f\n",
                           sever_one, four_five_six_one, sever_two, four_five_six_two, 1.0);
                    fclose(fp);

                    // ofs << sever_one << " " << four_five_six_one << " " << sever_two << " " << four_five_six_two << " " << sever_three << "\n";
                    FILE *fp2 = fopen("result.txt", "a+");
                    if (fp2 == nullptr) {
                        exit(0);
                    }
                    fprintf(fp2,
                            "Calculate! sever_one: %f, four_five_six_one: %f, sever_two: %f, four_five_six_two: %f, sever_three: %f\n",
                            sever_one, four_five_six_one, sever_two, four_five_six_two, 1.0);
                    fflush(fp2);
                    // sleep(2);
                    system("bash bash.sh >> result.txt");
                }
            }
        }
    }
    // fclose(fp2);

    return 0;
}