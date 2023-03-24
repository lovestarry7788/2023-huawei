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

    for(double sever_one = 1.0; sever_one <= 1.6; sever_one += 0.1) {
        for(double four_five_six_one = 1.0; four_five_six_one <= 4.1; four_five_six_one += 0.1) {
            for(double sever_two = 1.0; sever_two <= 1.55; sever_two += 0.1) {
                for (double four_five_six_two = 1.0; four_five_six_two <= 1.55; four_five_six_two += 0.1) {
                    FILE *fp = fopen("config.txt", "w+");
                    fprintf(fp, "%f %f %f %f %f\n", sever_one, four_five_six_one, sever_two, four_five_six_two, 1.0);
                    printf("Calculate! sever_one: %f, four_five_six_one: %f, sever_two: %f, four_five_six_two: %f, sever_three: %f\n",
                           sever_one, four_five_six_one, sever_two, four_five_six_two, 1.0);
//                    fprintf(fp, "%f %f\n", one, two);
//                    printf("Calculate! one: %f, two: %f\n", one, two);
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
                    system("./Robot -m pre_official_map/2.txt -f ./SDK/c++/build/main >> result.txt");
                }
            }
        }
    }
    // fclose(fp2);

    return 0;
}