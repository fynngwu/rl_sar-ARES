//
// 跳跃轨迹测试: 加载极坐标关键帧，打印每帧的笛卡尔位置和IK解算结果
// 编译: g++ -std=c++17 -I src/rl_sar/include test_jump.cpp -o test_jump -lyaml-cpp
// 运行: ./test_jump
//

#include "jump_controller.hpp"
#include <cstdio>
#include <cmath>
#include <array>

static constexpr int MOTOR_IDX[4][3] = {{0,4,8},{2,6,10},{1,5,9},{3,7,11}};

int main()
{
    printf("=== Jump Trajectory Test ===\n\n");

    JumpController jc;
    jc.LoadFromYaml("policy", "position_control");

    jc.Reset();

    printf("\n=== Simulate driver loop ===\n");
    const char* leg_names[] = {"FL", "RL", "FR", "RR"};
    int step = 0;
    std::array<float, 12> last_pos{};
    while (!jc.IsDone()) {
        auto pos = jc.Update();
        last_pos = pos;
        if (step % 5 == 0) {
            printf("[%2d] ", step);
            for (int leg = 0; leg < 4; ++leg)
                printf("%s(%+7.4f,%+7.4f,%+7.4f) ",
                       leg_names[leg], pos[MOTOR_IDX[leg][0]], pos[MOTOR_IDX[leg][1]], pos[MOTOR_IDX[leg][2]]);
            printf("\n");
        }
        step++;
    }
    printf("Done at step %d\n\n", step);

    // 打印最终轨迹点 (落地) 的 motor 值
    printf("=== Final landing motor targets ===\n");
    printf("Motor:   0    1    2    3    4    5    6    7    8    9   10   11\n");
    printf("Joint: --abad--      ----hip----       ---knee---\n");
    printf("Leg:   LF   LH   RF   RH   LF   LH   RF   RH   LF   LH   RF   RH\n");
    printf("Group: F    R    F    R    F    R    F    R    F    R    F    R\n");
    printf("Rad:  ");
    for (int j = 0; j < 12; ++j)
        printf("%6.3f ", last_pos[j]);
    printf("\n");
    printf("Deg:  ");
    for (int j = 0; j < 12; ++j)
        printf("%+6.1f ", last_pos[j] * RAD2DEG);
    printf("\n");

    return 0;
}
