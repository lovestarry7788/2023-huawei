
代码位置:`/SDK/c++/main.cpp`

#### 1. 生成可执行文件 `./main`
>cd /SDK/c++  
>cmake .  
>make

#### 2. 判题器使用 simple
> ./Robot -m maps/1.txt -c ./SDK/c++ "./main"

#### 2.1 参数解释

> robot [option...] <player's program>

`必选参数`  
-m 指定地图文件  
-f 快速模式，不按照自然时间运行，选手返回控制指令就提前进入下一帧  
-d 调试模式，不限制选手的初始化和每帧运行时间，方便选手挂载调试器

`可选参数`  
-c 指定选手程序的当前目录
