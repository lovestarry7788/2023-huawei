#ifndef HW2023_LOG_H
#define HW2023_LOG_H

#include "input.h"
#include <fstream>
#include <vector>
#include <string_view>
#include <iostream>

namespace Log {
    extern std::ofstream ofs;
    template<class T, class... A> void print(T&& x, A&&... a){ 
        // if (Input::frameID != 1954) return;
        ofs<<x; (int[]){(ofs<< ' '<< a,0)...}; ofs<<'\n'; 
        ofs.flush();
    }
}

#endif