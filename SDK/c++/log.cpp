#include "log.h"

using namespace Log;


// template<class T, class... A> void Log::print(T&& x, A&&... a) { 
//     ofs<<x; (int[]){(ofs<< ' '<< a,0)...}; ofs<<'\n'; 
// }
std::ofstream Log::ofs("main.log");