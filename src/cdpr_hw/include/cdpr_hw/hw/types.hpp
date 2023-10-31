#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <unordered_map>

namespace cdpr_hw
{
struct ActCoeff
{
    double act2pos, act2vel, act2effort, pos2act, vel2act, effort2act, max_out, act2pos_offset, act2vel_offset,
        act2effort_offset, kp, kw, mode;
};

struct ActData
{
    std::string name;
    std::string type;
    ros::Time stamp;
    bool halted = false;
    int seq;
    int temp;
    double pos_last;
    double frequency;
    double pos, vel, effort;
    double cmd_pos, cmd_vel, cmd_effort, exe_effort;
    double zero_point;
};

struct ActDataPtr
{
    std::unordered_map<std::string, ActCoeff>* type2act_coeffs_;
    std::unordered_map<int, ActData>* id2act_data_;
};

struct VectorHasher
{
    int operator()(const std::vector<int>& V) const
    {
        int hash = V.size();
        for (auto& i : V)
        {
            hash ^= i + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};
}  // namespace cdpr_hw