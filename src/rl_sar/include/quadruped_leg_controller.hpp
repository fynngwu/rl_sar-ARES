#pragma once

#include <cmath>
#include <array>
#include <string>
#include <algorithm>
#include <cstdio>
#include <Eigen/Dense>

constexpr double PI = 3.14159265358979323846;
constexpr double RAD2DEG = 180.0 / PI;

struct Vec3 { double x, y, z; };

struct GaitParams {
    double period      = 0.8;     // gait period [s]
    double duty_factor = 0.5;     // stance ratio
    double step_height = 0.10;    // swing lift [m]
    double max_stride  = 0.18;    // max half-stride [m]
    double stride_scale = 1.0;     // runtime stride multiplier
    double foot_center_x = -0.05;  // neutral foot x in hip frame [m]
};

struct GaitCommand {
    double vx = 0.0;  // forward [m/s]
    double vy = 0.0;  // lateral [m/s]
    double wz = 0.0;  // yaw [rad/s]
};

struct LegState {
    Vec3 foot;               // foot position in hip frame
    double phase = 0.0;      // [0,1)
    bool is_swing = false;
    Eigen::Vector3d q;       // [abad, hip, knee] in rad
    Eigen::Vector3d q_deg;   // in deg
};

// Leg index convention: 0=LF, 1=RF, 2=LH, 3=RH
class QuadrupedLegController {
public:
    struct LegConfig {
        std::string name;
        Eigen::Vector3d hip_pos;        // hip origin in body frame
        double phase_offset = 0.0;      // trot phase offset
    };

    QuadrupedLegController() {
        L2_ = 0.21;     // thigh
        L3_ = 0.24;     // shank

        stand_height_ = 0.3;

        // Workspace limits (hip frame)
        x_min_ = -0.15;  x_max_ = 0.15;
        y_min_ = -0.1;   y_max_ = 0.1;
        r_min_ = 0.15;   r_max_ = 0.38;
        z_min_ = -0.35;   z_max_ = -0.15;

        // Joint offsets: left legs and right legs have opposite signs
        joint_offsets_ <<  0.0,   0.0,   0.0,   0.0,    // abad:  LF, RF, LH, RH
                          -1.0,   1.0,  -1.0,   1.0,    // hip:   LF, RF, LH, RH
                           1.5,  -1.5,   1.5,  -1.5;    // knee:  LF, RF, LH, RH

        // Joint direction signs: +1 normal, -1 reversed per joint per leg
        joint_signs_ <<  1,  1,  -1,  -1,    // abad:  LF, RF, LH, RH
                         -1,  1,  -1,  1,    // hip:   LF, RF, LH, RH
                         -1,  1,  -1,  1;    // knee:  LF, RF, LH, RH

        // Leg configs: LF, RF, LH, RH
        legs_ = {{
            {"LF", { 0.16,  0.0675, 0.0}, 0.0},
            {"RF", { 0.16, -0.0675, 0.0}, 0.5},
            {"LH", {-0.16,  0.0675, 0.0}, 0.5},
            {"RH", {-0.16, -0.0675, 0.0}, 0.0},
        }};

        // Internal time: default start at 0.5s, dt=0.02 (50Hz)
        t_ = 0.5;
        dt_ = 0.02;
    }

    void set_gait(const GaitParams& g) { gait_ = g; }
    void set_stand_height(double h) { stand_height_ = h; }
    void set_dt(double dt) { dt_ = dt; }
    double dt() const { return dt_; }

    void reset() { t_ = 0.5; }

    // Update all 4 legs using internal time, return state array
    std::array<LegState, 4> update(const GaitCommand& cmd) {
        auto states = update(t_, cmd);
        t_ += dt_;
        return states;
    }

    // Update all 4 legs with explicit time
    std::array<LegState, 4> update(double t, const GaitCommand& cmd) {
        std::array<LegState, 4> states;
        for (int i = 0; i < 4; ++i) {
            states[i] = compute_leg(t, i, cmd);
        }
        return states;
    }

    // Compute one leg
    LegState compute_leg(double t, int leg_idx, const GaitCommand& cmd) const {
        const auto& leg = legs_[leg_idx];

        // Phase
        double phase = std::fmod(t / gait_.period + leg.phase_offset, 1.0);
        if (phase < 0.0) phase += 1.0;
        bool swing = (phase >= gait_.duty_factor);

        // Foot target in hip frame
        Vec3 foot = gen_foot_hip(t, leg, cmd);

        // Solve IK (clamps out-of-workspace to limits)
        Eigen::Vector3d q = solve_ik(foot);
        Eigen::Vector3d q_motor = to_motor_angles(leg_idx, q);

        LegState s;
        s.foot   = foot;
        s.phase  = phase;
        s.is_swing = swing;
        s.q         = q_motor;
        s.q_deg     = q_motor * RAD2DEG;
        return s;
    }

    // Foot trajectory in hip frame
    Vec3 gen_foot_hip(double t, const LegConfig& leg, const GaitCommand& cmd) const {
        Vec3 foot;
        foot.x = gait_.foot_center_x;
        foot.y = 0.0;
        foot.z = -stand_height_;

        bool standing =
            std::abs(cmd.vx) < 0.05 &&
            std::abs(cmd.vy) < 0.05 &&
            std::abs(cmd.wz) < 0.05;

        if (standing) {
            return foot;
        }

        // Velocity in body frame, then subtract hip to get leg velocity
        double vx_leg = cmd.vx - cmd.wz * leg.hip_pos.y();
        double vy_leg = cmd.vy + cmd.wz * leg.hip_pos.x();

        double stride_x = clamp(vx_leg * gait_.period * gait_.duty_factor * gait_.stride_scale,
                                -gait_.max_stride, gait_.max_stride);
        double stride_y = clamp(vy_leg * gait_.period * gait_.duty_factor * gait_.stride_scale,
                                -gait_.max_stride, gait_.max_stride);

        double phase = std::fmod(t / gait_.period + leg.phase_offset, 1.0);
        if (phase < 0.0) phase += 1.0;

        if (phase < gait_.duty_factor) {
            double s = phase / gait_.duty_factor;
            foot.x += stride_x * (0.5 - s);
            foot.y += stride_y * (0.5 - s);
        } else {
            double s = (phase - gait_.duty_factor) / (1.0 - gait_.duty_factor);
            double c = s - std::sin(2.0 * PI * s) / (2.0 * PI);
            foot.x += stride_x * (-0.5 + c);
            foot.y += stride_y * (-0.5 + c);
            foot.z += gait_.step_height * 0.5 * (1.0 - std::cos(2.0 * PI * s));
        }
        return foot;
    }

    // 3-DOF IK: foot_hip -> [q_abad, q_hip, q_knee]
    // Out-of-workspace values are clamped to limits before solving
    Eigen::Vector3d solve_ik(const Vec3& p_hip) const {
        double x = clamp(p_hip.x, x_min_, x_max_);
        double y = clamp(p_hip.y, y_min_, y_max_);
        double z = clamp(p_hip.z, z_min_, z_max_);

        

        // q1: abduction
        double q1 = std::atan2(y, -z);

        // Project into pitch plane
        double z_plane = -std::sqrt(y * y + z * z);

        // Knee
        double r2 = x * x + z_plane * z_plane;
        double cos_q3 = clamp((r2 - L2_ * L2_ - L3_ * L3_) / (2.0 * L2_ * L3_), -1.0, 1.0);
        double q3 = std::acos(cos_q3);

        // Hip
        double alpha = std::atan2(x, -z_plane);
        double beta  = std::atan2(L3_ * std::sin(q3), L2_ + L3_ * std::cos(q3));
        double q2 = alpha - beta;

        return {q1, q2, q3};
    }

    // Accessors
    double L2() const { return L2_; }
    double L3() const { return L3_; }
    double stand_height() const { return stand_height_; }
    double time() const { return t_; }
    const Eigen::Matrix<double, 3, 4>& joint_offsets() const { return joint_offsets_; }
    const Eigen::Matrix<double, 3, 4>& joint_signs() const { return joint_signs_; }
    const LegConfig& leg(int i) const { return legs_[i]; }

    Eigen::Vector3d to_motor_angles(int leg_idx, const Eigen::Vector3d& q_ik) const {
        return joint_signs_.col(leg_idx).cwiseProduct(q_ik)
             + joint_offsets_.col(leg_idx);
    }

private:
    double L2_, L3_;
    double stand_height_;
    double x_min_, x_max_, y_min_, y_max_;
    double r_min_, r_max_, z_min_, z_max_;
    Eigen::Matrix<double, 3, 4> joint_offsets_;  // [abad,hip,knee] x [LF,RF,LH,RH]
    Eigen::Matrix<double, 3, 4> joint_signs_;    // +1 normal, -1 reversed per joint per leg
    std::array<LegConfig, 4> legs_;
    GaitParams gait_;

    // Internal time management
    double t_;
    double dt_;

    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }
};
