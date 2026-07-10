#pragma once

#include "quadruped_leg_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

class ClimbController {
public:
    static constexpr int NUM_JOINTS = 12;

    bool LoadFromYaml(const std::string& policy_dir, const std::string& policy_name)
    {
        std::string path = policy_dir + "/" + policy_name + "/config.yaml";
        try {
            YAML::Node climb = YAML::LoadFile(path)[policy_name]["climb"];
            if (!climb || !climb["phases"]) {
                printf("[CLIMB] No climb config\n");
                return false;
            }

            dt_ = LoadDoubleOrDefault(climb, "dt", dt_);
            std::array<Vec3, 4> prev = LoadFeetOrDefault(climb["start_feet"], NeutralFeet());
            points_.clear();

            for (const auto& phase : climb["phases"]) {
                Phase p;
                p.name = phase["name"] ? phase["name"].as<std::string>() : "unnamed";
                p.duration = phase["duration"].as<double>();
                p.smooth = !phase["mode"] || phase["mode"].as<std::string>() != "step";
                p.feet = LoadFeetOrDefault(phase["feet"], prev);

                int steps = std::max(1, static_cast<int>(std::round(p.duration / dt_)));
                for (int s = 0; s < steps; ++s) {
                    double u = static_cast<double>(s + 1) / steps;
                    double a = p.smooth ? SmoothStep(u) : 1.0;
                    Point point;
                    point.name = p.name;
                    for (int leg = 0; leg < 4; ++leg) {
                        Vec3 foot = Lerp(prev[leg], p.feet[leg], a);
                        auto q = leg_ctrl_.to_motor_angles(leg, leg_ctrl_.solve_ik(foot));
                        for (int j = 0; j < 3; ++j)
                            point.pos[MOTOR_IDX[leg][j]] = static_cast<float>(q(j));
                    }
                    points_.push_back(point);
                }
                prev = p.feet;
            }

            loaded_ = !points_.empty();
            printf("[CLIMB] Loaded %zu points from %s\n", points_.size(), path.c_str());
            return loaded_;
        } catch (const std::exception& e) {
            loaded_ = false;
            points_.clear();
            printf("[CLIMB] Load error: %s\n", e.what());
            return false;
        }
    }

    bool Start()
    {
        if (!loaded_ || points_.empty()) {
            active_ = false;
            step_ = 0;
            return false;
        }
        active_ = true;
        step_ = 0;
        last_phase_.clear();
        return true;
    }

    void Stop()
    {
        active_ = false;
        step_ = points_.size();
    }

    bool IsLoaded() const { return loaded_; }
    bool IsActive() const { return active_; }
    bool IsDone() const { return step_ >= points_.size(); }
    const std::string& LastPhase() const { return last_phase_; }

    std::array<float, NUM_JOINTS> Update()
    {
        if (!active_ || points_.empty()) return {};
        if (step_ >= points_.size()) {
            active_ = false;
            return points_.back().pos;
        }
        const Point& point = points_[step_++];
        last_phase_ = point.name;
        if (step_ >= points_.size())
            active_ = false;
        return point.pos;
    }

private:
    struct Phase {
        std::string name;
        double duration = 0.0;
        bool smooth = true;
        std::array<Vec3, 4> feet{};
    };

    struct Point {
        std::array<float, NUM_JOINTS> pos{};
        std::string name;
    };

    static constexpr int MOTOR_IDX[4][3] = {{0,4,8},{2,6,10},{1,5,9},{3,7,11}};

    static double LoadDoubleOrDefault(const YAML::Node& node, const char* key, double fallback)
    {
        return (node && node[key]) ? node[key].as<double>() : fallback;
    }

    static double SmoothStep(double x)
    {
        x = std::clamp(x, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    static Vec3 Lerp(const Vec3& a, const Vec3& b, double t)
    {
        return {
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t,
        };
    }

    static Vec3 LoadFootOrDefault(const YAML::Node& node, const Vec3& fallback)
    {
        if (!node) return fallback;
        Vec3 foot = fallback;
        if (node["x"]) foot.x = node["x"].as<double>();
        if (node["y"]) foot.y = node["y"].as<double>();
        if (node["z"]) foot.z = node["z"].as<double>();
        return foot;
    }

    static YAML::Node LegNode(const YAML::Node& feet, const char* a, const char* b)
    {
        if (!feet) return YAML::Node();
        if (feet[a]) return feet[a];
        if (feet[b]) return feet[b];
        return YAML::Node();
    }

    static std::array<Vec3, 4> LoadFeetOrDefault(const YAML::Node& feet,
                                                  const std::array<Vec3, 4>& fallback)
    {
        std::array<Vec3, 4> out = fallback;
        out[0] = LoadFootOrDefault(LegNode(feet, "LF", "FL"), fallback[0]);
        out[1] = LoadFootOrDefault(LegNode(feet, "RF", "FR"), fallback[1]);
        out[2] = LoadFootOrDefault(LegNode(feet, "LH", "RL"), fallback[2]);
        out[3] = LoadFootOrDefault(LegNode(feet, "RH", "RR"), fallback[3]);
        return out;
    }

    static std::array<Vec3, 4> NeutralFeet()
    {
        return {{
            {-0.05, 0.0, -0.30},
            {-0.05, 0.0, -0.30},
            {-0.05, 0.0, -0.30},
            {-0.05, 0.0, -0.30},
        }};
    }

    double dt_ = 0.02;
    bool loaded_ = false;
    bool active_ = false;
    size_t step_ = 0;
    std::string last_phase_;
    std::vector<Point> points_;
    QuadrupedLegController leg_ctrl_;
};
