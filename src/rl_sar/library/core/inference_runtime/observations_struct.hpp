#ifndef OBSERVATIONS_STRUCT_HPP
#define OBSERVATIONS_STRUCT_HPP

#include <vector>

template <typename T>
struct Observations
{
    std::vector<T> lin_vel;
    std::vector<T> ang_vel;
    std::vector<T> gravity_vec;
    std::vector<T> commands;
    std::vector<T> base_quat;
    std::vector<T> dof_pos;
    std::vector<T> dof_vel;
    std::vector<T> actions;
};

#endif
