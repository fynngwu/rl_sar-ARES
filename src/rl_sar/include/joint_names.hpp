#pragma once

#include <array>

static constexpr int kNumJoints = 12;

// Joint names by leg (FL RL FR RR order - used by ROS topics)
static constexpr const char* kJointNamesByLeg[kNumJoints] = {
    "fl_hipa", "fl_hipf", "fl_knee",
    "rl_hipa", "rl_hipf", "rl_knee",
    "fr_hipa", "fr_hipf", "fr_knee",
    "rr_hipa", "rr_hipf", "rr_knee"
};

// Joint names by DOF type (HipA HipF Knee order - used by internal driver)
static constexpr const char* kJointNamesByDof[kNumJoints] = {
    "FL_HipA", "RL_HipA", "FR_HipA", "RR_HipA",
    "FL_HipF", "RL_HipF", "FR_HipF", "RR_HipF",
    "FL_Knee", "RL_Knee", "FR_Knee", "RR_Knee"
};
