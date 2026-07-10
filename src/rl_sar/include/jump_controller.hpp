#pragma once
//
// JumpController — 跳跃轨迹控制器
//
// 工作原理:
//   1. 初始化时从 YAML 读取极坐标关键帧 (θ, r)
//   2. 极坐标转笛卡尔: x = r*sin(θ), z = -r*cos(θ)
//   3. 四条腿完全一致，同一位置 IK 求解后复制到四腿
//   4. 按控制周期(20ms)展开成连续轨迹点，存入 points_[]
//   5. 运行时: 每次 Update() 计数器+1，直接按索引取点
//
// 极坐标说明:
//   θ = 从竖直向下算的角度 [度]
//       正值 = 腿向前, 负值 = 腿向后
//   r = 足端到髋关节的距离 [m]
//       r 越大腿越伸展, r 越小腿越收缩
//

#include "quadruped_leg_controller.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

class JumpController {
public:
    static constexpr int NUM_JOINTS = 12;

    JumpController() = default;

    //
    // 从 YAML 加载跳跃轨迹并预计算全部轨迹点
    //
    // YAML 格式:
    //   jump:
    //     keyframes:
    //       - theta: -30       # 角度 [度], 竖直向下=0°, 正=前, 负=后 (四腿统一)
    //         r: 0.18          # 足端到髋距离 [m] (四腿统一)
    //         duration: 0.10   # 该阶段持续时间 [s]
    //         mode: smooth     # smooth=平滑插值, step=瞬跳
    //       - theta_front: 20  # 前腿(FL,FR)角度, 可选, 不设则用 theta
    //         theta_rear: -30  # 后腿(RL,RR)角度, 可选, 不设则用 theta
    //         r_front: 0.15    # 前腿距离, 可选, 不设则用 r
    //         r_rear: 0.20     # 后腿距离, 可选, 不设则用 r
    //         duration: 0.10
    //         mode: smooth
    //
    // 返回 true = 加载成功
    //
    bool LoadFromYaml(const std::string& policy_dir, const std::string& policy_name)
    {
        std::string path = policy_dir + "/" + policy_name + "/config.yaml";
        try {
            YAML::Node root = YAML::LoadFile(path)[policy_name]["jump"];
            if (!root || !root["keyframes"]) {
                printf("[JUMP] No jump config\n");
                return false;
            }

            // ---- 读取极坐标关键帧 ----
            struct PolarKF { double theta_front, theta_rear, r_front, r_rear, dur; bool smooth; };
            std::vector<PolarKF> pks;
            for (const auto& kf : root["keyframes"]) {
                double theta = kf["theta"].as<double>(0.0);
                double tf = kf["theta_front"].as<double>(theta);
                double tr = kf["theta_rear"].as<double>(theta);
                double r = kf["r"].as<double>(0.18);
                double rf = kf["r_front"].as<double>(r);
                double rr = kf["r_rear"].as<double>(r);
                pks.push_back({tf, tr, rf, rr,
                               kf["duration"].as<double>(),
                               kf["mode"].as<std::string>() == "smooth"});
            }

            // ---- 极坐标 → 笛卡尔 → IK → 12个电机位置 ----
            // 前腿(FL=0, FR=2) 和 后腿(RL=1, RR=3) 分别用不同角度和距离
            auto polar_to_motors = [&](double theta_front_deg, double theta_rear_deg,
                                       double r_front, double r_rear) {
                std::array<float, NUM_JOINTS> m{};
                // 前腿 LF(0), RF(1)
                double rad_f = theta_front_deg * M_PI / 180.0;
                Vec3 foot_f = {r_front * std::sin(rad_f), 0.0, -r_front * std::cos(rad_f)};
                for (int leg : {0, 1}) {
                    auto q = leg_ctrl_.to_motor_angles(leg, leg_ctrl_.solve_ik(foot_f));
                    for (int j = 0; j < 3; ++j)
                        m[MOTOR_IDX[leg][j]] = (float)q(j);
                }
                // 后腿 LH(2), RH(3)
                double rad_r = theta_rear_deg * M_PI / 180.0;
                Vec3 foot_r = {r_rear * std::sin(rad_r), 0.0, -r_rear * std::cos(rad_r)};
                for (int leg : {2, 3}) {
                    auto q = leg_ctrl_.to_motor_angles(leg, leg_ctrl_.solve_ik(foot_r));
                    for (int j = 0; j < 3; ++j)
                        m[MOTOR_IDX[leg][j]] = (float)q(j);
                }
                return m;
            };

            // ---- 预计算连续轨迹点 ----
            // 每个关键帧按 20ms 步长展开:
            //   smooth 帧: 从上一帧线性插值到当前帧
            //   step 帧:   直接跳到当前帧，全程保持
            points_.clear();
            double t = 0.0;
            double dt = 0.02;
            for (size_t i = 0; i < pks.size(); ++i) {
                auto cur = polar_to_motors(pks[i].theta_front, pks[i].theta_rear,
                                           pks[i].r_front, pks[i].r_rear);
                int n = std::max(1, (int)(pks[i].dur / dt));
                for (int s = 0; s < n; ++s) {
                    float a = pks[i].smooth ? (float)s / n : 1.0f;
                    auto prev = polar_to_motors(pks[i > 0 ? i - 1 : i].theta_front,
                                                pks[i > 0 ? i - 1 : i].theta_rear,
                                                pks[i > 0 ? i - 1 : i].r_front,
                                                pks[i > 0 ? i - 1 : i].r_rear);
                    Point p;
                    for (int j = 0; j < NUM_JOINTS; ++j)
                        p.pos[j] = prev[j] + (cur[j] - prev[j]) * a;
                    points_.push_back(p);
                }
                t += pks[i].dur;
            }
            // 追加终点
            Point ep; ep.pos = polar_to_motors(pks.back().theta_front, pks.back().theta_rear,
                                               pks.back().r_front, pks.back().r_rear);
            points_.push_back(ep);
            printf("[JUMP] %zu keyframes → %zu points (%.3fs)\n",
                   pks.size(), points_.size(), t);
            return true;
        } catch (const std::exception& e) {
            printf("[JUMP] Load error: %s\n", e.what());
            return false;
        }
    }

    // 重置计数器并激活轨迹
    void Reset() { active_ = true; step_ = 0; }

    bool IsActive() const { return active_; }

    bool IsDone() const { return step_ >= points_.size(); }

    // 每帧调用: 返回当前轨迹点的12个电机位置，计数器+1
    std::array<float, NUM_JOINTS> Update()
    {
        if (!active_ || points_.empty()) return {};
        if (step_ >= points_.size()) { active_ = false; return points_.back().pos; }
        return points_[step_++].pos;
    }

private:
    static constexpr int MOTOR_IDX[4][3] = {{0,4,8},{2,6,10},{1,5,9},{3,7,11}};

    struct Point { std::array<float, NUM_JOINTS> pos{}; };

    std::vector<Point> points_;       // 预计算的完整轨迹
    bool active_ = false;             // 是否正在执行
    size_t step_ = 0;                 // 当前轨迹点索引，每次 Update +1
    QuadrupedLegController leg_ctrl_; // 仅初始化时用于 IK 求解
};
