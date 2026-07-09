//
// 跳跃轨迹测试: 加载极坐标关键帧，打印每帧的笛卡尔位置和IK解算结果
// 编译: g++ -std=c++17 -I src/rl_sar/include test_jump.cpp -o test_jump -lyaml-cpp
// 运行: ./test_jump
//

#include "jump_controller.hpp"
#include <cstdio>
#include <cmath>

int main()
{
    printf("=== Jump Trajectory Test ===\n\n");

    JumpController jc;
    jc.LoadFromYaml("policy", "dogv2_cts/cts");

    // 模拟 driver 调用: Reset 激活轨迹
    jc.Reset();

    printf("\n=== Simulate driver loop ===\n");
    int step = 0;
    while (!jc.IsDone()) {
        auto pos = jc.Update();
        if (step % 5 == 0) {  // 每5步打印一次 (100ms间隔)
            printf("[%2d] ", step);
            int legs[4][3] = {{0,4,8},{2,6,10},{1,5,9},{3,7,11}};
            for (int leg = 0; leg < 4; ++leg)
                printf("(%+6.3f,%+6.3f,%+6.3f) ",
                       pos[legs[leg][0]], pos[legs[leg][1]], pos[legs[leg][2]]);
            printf("\n");
        }
        step++;
    }
    printf("Done at step %d (IsDone=%d, IsActive=%d)\n",
           step, jc.IsDone(), jc.IsActive());
}
