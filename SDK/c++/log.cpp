#include "log.h"

<<<<<<< HEAD
using namespace Log;


// template<class T, class... A> void Log::print(T&& x, A&&... a) { 
//     ofs<<x; (int[]){(ofs<< ' '<< a,0)...}; ofs<<'\n'; 
// }
=======
>>>>>>> 将Input与output分到文件中
std::ofstream Log::ofs("main.log");