#pragma once
//
// JumpController — 跳跃轨迹控制器
//
// ============================================================
// 腿的编号 (来自 quadruped_leg_controller.hpp)
// ============================================================
//   leg 0 = LF (Left Front,  左前) = FL
//   leg 1 = RF (Right Front, 右前) = FR
//   leg 2 = LH (Left Hind,   左后) = RL
//   leg 3 = RH (Right Hind,  右后) = RR
//
// ============================================================
// 电机编号 (来自 driver 层 MOTOR_IDX)
// ============================================================
//   每条腿有3个电机: [HipA侧摆, HipF前后摆, Knee膝关节]
//
//   leg 0 (LF/FL): 电机 0, 4, 8     ← 位置 pos[0], pos[4], pos[8]
//   leg 1 (RF/FR): 电机 2, 6, 10    ← 位置 pos[2], pos[6], pos[10]
//   leg 2 (LH/RL): 电机 1, 5, 9     ← 位置 pos[1], pos[5], pos[9]
//   leg 3 (RH/RR): 电机 3, 7, 11    ← 位置 pos[3], pos[7], pos[11]
//
//   注意: 电机编号不是按腿顺序排的！
//   pos 数组的 12 个位置按 "HipA[0-3], HipF[4-7], Knee[8-11]" 分组
//   所以 leg0 的 hip 在 pos[0]，leg1 的 hip 在 pos[2]，以此类推
//
// ============================================================
// 前后腿分组
// ============================================================
//   "front" 组 = leg 0 (LF) + leg 1 (RF) = 左前 + 右前
//   "rear"  组 = leg 2 (LH) + leg 3 (RH) = 左后 + 右后
//
//   YAML 中 front/rear 各自写独立的关键帧列表
//   每组的关键帧可以有不同的持续时间、不同的数量
//   比如: 后腿先蹬地(0.1s)，前腿再跟上(0.3s)
//
// ============================================================
// 工作流程
// ============================================================
//   1. LoadFromYaml(): 从 YAML 读取 front/rear 两组关键帧
//   2. 每组独立计算轨迹: 极坐标(θ,r) → 笛卡尔(x,z) → IK求解 → 电机角度
//   3. 两组轨迹按 20ms 步长展开，取较长的那组决定总时长
//      短的那组在多出来的时间里保持最后的位置不动
//   4. Update(): 每帧调用，把 front 6个电机 + rear 6个电机
//      合并成完整的 12 电机位置数组返回
//
// 极坐标说明:
//   θ = 从竖直向下算的角度 [度]
//       正值 = 腿向前伸, 负值 = 腿向后收
//   r = 足端到髋关节的距离 [m]
//       r 越大 → 腿越伸展(站高), r 越小 → 腿越收缩(蹲低)
//   y = 侧向偏移量 [m] (可选, 默认 0)
//       正值 = 腿向外张开, 左右对称:
//         左腿(LF, LH) → foot_y = +y (向左张)
//         右腿(RF, RH) → foot_y = -y (向右张)
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
    // YAML 格式示例:
    //   jump:
    //     dt: 0.02                          # 轨迹分辨率 [s], 每 20ms 一个点
    //     front:                            # ===== 前腿(FL+FR)的关键帧 =====
    //       - theta: -10                    #   起始姿态: 腿向后收 10°
    //         r: 0.3                        #   腿长 0.3m
    //         duration: 0.1                 #   这个阶段持续 0.1 秒
    //         mode: smooth                  #   smooth=平滑过渡, step=瞬间跳到目标
    //       - theta: -30                    #   蓄力: 前腿向后收更多
    //         r: 0.13
    //         duration: 0.3
    //         mode: smooth
    //       - theta: 30                     #   蹬地: 前腿向前踢出
    //         r: 0.2
    //         duration: 0.1
    //         mode: smooth
    //     rear:                             # ===== 后腿(RL+RR)的关键帧 =====
    //       - theta: -10
    //         r: 0.3
    //         duration: 0.1
    //         mode: smooth
    //       - theta: -40                    #   后腿蹬地角度更大
    //         r: 0.4
    //         y: 0.06                       #   侧向偏移: 左后腿+y 右后腿-y (张开胯部)
    //         duration: 0.1                 #   后腿蹬地更快(0.1s vs 前腿0.3s)
    //         mode: step
    //       - theta: -10                    #   落地: 后腿收回
    //         r: 0.14
    //         y: 0.0                        #   收回胯部
    //         duration: 0.1
    //         mode: smooth
    //
    // y 参数说明 (可选, 默认 0):
    //   正值 = 腿向外张开, 左右对称:
    //     左腿(LF, LH) → foot_y = +y (向左张)
    //     右腿(RF, RH) → foot_y = -y (向右张)
    //
    // 关键点: front 和 rear 的 duration 可以完全不同！
    //   比如上面 front 总共 0.1+0.3+0.1=0.5s, rear 总共 0.1+0.1+0.1=0.3s
    //   最终轨迹长度取 0.5s (较长的那个), rear 在最后 0.2s 保持落地姿态不动
    //
    // 返回 true = 加载成功
    //
    bool LoadFromYaml(const std::string& policy_dir, const std::string& policy_name)
    {
        std::string path = policy_dir + "/" + policy_name + "/config.yaml";
        try {
            YAML::Node root = YAML::LoadFile(path)[policy_name]["jump"];
            if (!root) {
                printf("[JUMP] No jump config\n");
                return false;
            }

            dt_ = root["dt"].as<double>(0.02);

            // 一个关键帧: 角度 + 腿长 + 侧向偏移 + 持续时间 + 是否平滑
            struct PolarKF { double theta, r, y, dur; bool smooth; };

            // 解析一组腿的 YAML 关键帧列表
            auto parse_group = [&](const std::string& key) -> std::vector<PolarKF> {
                std::vector<PolarKF> kfs;
                YAML::Node node = root[key];
                if (!node || !node.IsSequence()) return kfs;
                for (const auto& kf : node) {
                    kfs.push_back({
                        kf["theta"].as<double>(0.0),
                        kf["r"].as<double>(0.18),
                        kf["y"].as<double>(0.0),
                        kf["duration"].as<double>(),
                        kf["mode"].as<std::string>() == "smooth"
                    });
                }
                return kfs;
            };

            std::vector<PolarKF> front_kfs = parse_group("front");
            std::vector<PolarKF> rear_kfs = parse_group("rear");

            if (front_kfs.empty() && rear_kfs.empty()) {
                printf("[JUMP] No front/rear keyframes\n");
                return false;
            }

            // ------------------------------------------------
            // 极坐标 → 电机角度
            // ------------------------------------------------
            // 输入: theta(角度), r(腿长), y(侧向偏移), leg_ids(要算哪几条腿)
            // 输出: 6个电机角度 (2条腿 × 3个关节)
            //
            // 转换过程:
            //   极坐标 (θ, r) + 侧向偏移 y → 笛卡尔坐标 (x, y, z)
            //     x = r * sin(θ)    ← 水平方向, 正=前 负=后
            //     z = -r * cos(θ)   ← 垂直方向, 负=向下(地面方向)
            //     y = ±y_offset      ← 侧向, 左腿正 右腿负 (左右对称张开)
            //   然后用 IK (逆运动学) 解算出 3 个关节角度
            //
            // leg_ids 的含义:
            //   front 组传 {0, 1} → 算 leg0(LF) 和 leg1(RF)
            //   rear  组传 {2, 3} → 算 leg2(LH) 和 leg3(RH)
            //
            // y 的符号:
            //   leg 0 (LF), leg 2 (LH) → 左腿 → foot_y = +y (向外张)
            //   leg 1 (RF), leg 3 (RH) → 右腿 → foot_y = -y (向外张)
            //
            // 输出数组 m[6] 的布局:
            //   m[0,1,2] = 第一条腿的 [HipA, HipF, Knee]
            //   m[3,4,5] = 第二条腿的 [HipA, HipF, Knee]
            // ------------------------------------------------
            auto polar_to_group_motors = [&](double theta_deg, double r, double y,
                                            const std::vector<int>& leg_ids) {
                std::array<float, 6> m{};
                double rad = theta_deg * M_PI / 180.0;
                for (size_t k = 0; k < leg_ids.size(); ++k) {
                    int leg = leg_ids[k];
                    // 左腿(0,2): +y, 右腿(1,3): -y → 左右对称张开
                    double y_sign = (leg % 2 == 0) ? 1.0 : -1.0;
                    Vec3 foot = {r * std::sin(rad), y_sign * y, -r * std::cos(rad)};
                    auto q = leg_ctrl_.to_motor_angles(leg, leg_ctrl_.solve_ik(foot));
                    for (int j = 0; j < 3; ++j)
                        m[k * 3 + j] = (float)q(j);
                }
                return m;
            };

            // ------------------------------------------------
            // 把一组关键帧展开成连续的轨迹点
            // ------------------------------------------------
            // 输入: 这组腿的关键帧列表 + 腿编号
            // 输出: 每 20ms 一个点, 每个点是 6 个电机角度
            //
            // 举例: 如果关键帧是 0.3s duration + smooth
            //   → 展开成 15 个点 (0.3 / 0.02 = 15)
            //   → 每个点从前一个关键帧线性插值到当前关键帧
            //
            // 如果是 step 模式:
            //   → 第一个点就直接跳到目标值, 后面保持不变
            // ------------------------------------------------
            auto compute_group_trajectory = [&](const std::vector<PolarKF>& kfs,
                                                const std::vector<int>& leg_ids) {
                std::vector<std::array<float, 6>> pts;
                if (kfs.empty()) return pts;

                for (size_t i = 0; i < kfs.size(); ++i) {
                    // 当前关键帧的目标电机角度
                    auto cur = polar_to_group_motors(kfs[i].theta, kfs[i].r, kfs[i].y, leg_ids);
                    // 该阶段需要多少个 20ms 步
                    int n = std::max(1, (int)(kfs[i].dur / dt_));
                    for (int s = 0; s < n; ++s) {
                        // a 是插值系数: smooth 从 0→1 线性增长, step 直接为 1
                        float a = kfs[i].smooth ? (float)s / n : 1.0f;
                        // 上一个关键帧的电机角度 (第一个关键帧用自身作为起点)
                        auto prev = polar_to_group_motors(
                            kfs[i > 0 ? i - 1 : i].theta,
                            kfs[i > 0 ? i - 1 : i].r,
                            kfs[i > 0 ? i - 1 : i].y, leg_ids);
                        // 线性插值: prev + (cur - prev) * a
                        std::array<float, 6> p{};
                        for (int j = 0; j < 6; ++j)
                            p[j] = prev[j] + (cur[j] - prev[j]) * a;
                        pts.push_back(p);
                    }
                }
                // 追加最终点 (确保轨迹精确停在最后一个关键帧)
                auto ep = polar_to_group_motors(kfs.back().theta, kfs.back().r, kfs.back().y, leg_ids);
                pts.push_back(ep);
                return pts;
            };

            // ------------------------------------------------
            // 分别计算前腿和后腿的轨迹
            // ------------------------------------------------
            // front: leg_ids = {0, 1} → LF(左前) + RF(右前)
            // rear:  leg_ids = {2, 3} → LH(左后) + RH(右后)
            //
            // 注意: 两组轨迹的点数可能不同！
            //   例如 front 有 25 个点 (0.5s), rear 有 15 个点 (0.3s)
            // ------------------------------------------------
            front_points_ = compute_group_trajectory(front_kfs, {0, 1});
            rear_points_ = compute_group_trajectory(rear_kfs, {2, 3});

            // ------------------------------------------------
            // 对齐两组轨迹的长度
            // ------------------------------------------------
            // 总长度取两组中较长的那个
            // 短的那组用最后一个点填充 (即保持不动)
            //
            // 举例:
            //   front 25个点, rear 15个点 → 总共 25 个点
            //   rear 的第 16~25 个点都等于第 15 个点 (保持落地姿态)
            // ------------------------------------------------
            size_t max_len = std::max(front_points_.size(), rear_points_.size());
            if (front_points_.size() < max_len && !front_points_.empty()) {
                auto last = front_points_.back();
                front_points_.resize(max_len, last);
            }
            if (rear_points_.size() < max_len && !rear_points_.empty()) {
                auto last = rear_points_.back();
                rear_points_.resize(max_len, last);
            }

            // 如果某一组完全没有关键帧 (比如只定义了 front), 用零值占位
            if (front_points_.empty()) {
                front_points_.resize(max_len, std::array<float, 6>{});
            }
            if (rear_points_.empty()) {
                rear_points_.resize(max_len, std::array<float, 6>{});
            }

            double total_t = (max_len - 1) * dt_;
            printf("[JUMP] front=%zu rear=%zu → %zu points (%.3fs, dt=%.3fs)\n",
                   front_points_.size() - 1, rear_points_.size() - 1,
                   max_len, total_t, dt_);
            return true;
        } catch (const std::exception& e) {
            printf("[JUMP] Load error: %s\n", e.what());
            return false;
        }
    }

    // 重置计数器并激活轨迹 (每次进入 JUMP 模式时调用)
    void Reset() { active_ = true; step_ = 0; }

    bool IsActive() const { return active_; }

    bool IsDone() const { return step_ >= front_points_.size(); }

    // ------------------------------------------------
    // 每帧调用: 返回当前轨迹点的 12 个电机位置
    // ------------------------------------------------
    // front_points_[step] 存了 6 个值: [LF的3关节, RF的3关节]
    // rear_points_[step]  存了 6 个值: [LH的3关节, RH的3关节]
    //
    // 这里把它们合并到 12 电机数组 pos[12] 中
    // 合并时要按 MOTOR_IDX 的映射关系填入正确的位置:
    //
    //   pos[0]  = LF_HipA  ← front[0]    pos[1]  = LH_HipA  ← rear[0]
    //   pos[2]  = RF_HipA  ← front[3]    pos[3]  = RH_HipA  ← rear[3]
    //   pos[4]  = LF_HipF  ← front[1]    pos[5]  = LH_HipF  ← rear[1]
    //   pos[6]  = RF_HipF  ← front[4]    pos[7]  = RH_HipF  ← rear[4]
    //   pos[8]  = LF_Knee ← front[2]    pos[9]  = LH_Knee ← rear[2]
    //   pos[10] = RF_Knee ← front[5]    pos[11] = RH_Knee ← rear[5]
    //
    // 简单记法: pos[电机号] = 对应腿的对应关节
    // ------------------------------------------------
    std::array<float, NUM_JOINTS> Update()
    {
        if (!active_ || front_points_.empty()) return {};
        if (step_ >= front_points_.size()) { active_ = false; return last_pos_; }

        std::array<float, NUM_JOINTS> pos{};

        // ---- 前腿 (front 组): LF(leg0) + RF(leg1) ----
        // front_points_ 布局: [LF_hipA, LF_hipF, LF_knee, RF_hipA, RF_hipF, RF_knee]
        // LF/leg0 的电机号: 0(HipA), 4(HipF), 8(Knee)
        // RF/leg1 的电机号: 2(HipA), 6(HipF), 10(Knee)
        pos[0]  = front_points_[step_][0];  // LF HipA
        pos[4]  = front_points_[step_][1];  // LF HipF
        pos[8]  = front_points_[step_][2];  // LF Knee
        pos[2]  = front_points_[step_][3];  // RF HipA
        pos[6]  = front_points_[step_][4];  // RF HipF
        pos[10] = front_points_[step_][5];  // RF Knee

        // ---- 后腿 (rear 组): LH(leg2) + RH(leg3) ----
        // rear_points_ 布局: [LH_hipA, LH_hipF, LH_knee, RH_hipA, RH_hipF, RH_knee]
        // LH/leg2 的电机号: 1(HipA), 5(HipF), 9(Knee)
        // RH/leg3 的电机号: 3(HipA), 7(HipF), 11(Knee)
        pos[1]  = rear_points_[step_][0];  // LH HipA
        pos[5]  = rear_points_[step_][1];  // LH HipF
        pos[9]  = rear_points_[step_][2];  // LH Knee
        pos[3]  = rear_points_[step_][3];  // RH HipA
        pos[7]  = rear_points_[step_][4];  // RH HipF
        pos[11] = rear_points_[step_][5];  // RH Knee

        last_pos_ = pos;
        ++step_;
        return pos;
    }

private:
    // 电机编号映射表: MOTOR_IDX[腿号][关节号] = 电机号
    // 腿 0 (LF/FL): 电机 0, 4, 8
    // 腿 1 (RF/FR): 电机 2, 6, 10
    // 腿 2 (LH/RL): 电机 1, 5, 9
    // 腿 3 (RH/RR): 电机 3, 7, 11
    static constexpr int MOTOR_IDX[4][3] = {{0,4,8},{2,6,10},{1,5,9},{3,7,11}};

    QuadrupedLegController leg_ctrl_;  // IK 求解器 (只在初始化时用)
    double dt_ = 0.02;                 // 轨迹时间步长 [s]

    // front_points_[i] = 第 i 个轨迹点的前腿 6 个电机角度
    //   布局: [LF_hipA, LF_hipF, LF_knee, RF_hipA, RF_hipF, RF_knee]
    std::vector<std::array<float, 6>> front_points_;

    // rear_points_[i] = 第 i 个轨迹点的后腿 6 个电机角度
    //   布局: [LH_hipA, LH_hipF, LH_knee, RH_hipA, RH_hipF, RH_knee]
    std::vector<std::array<float, 6>> rear_points_;

    bool active_ = false;       // 是否正在执行轨迹
    size_t step_ = 0;           // 当前执行到第几个轨迹点
    std::array<float, NUM_JOINTS> last_pos_{};  // 上一次输出的 12 电机位置
};
