#pragma once

#include <string>
#include <ros/ros.h>
#include <unordered_map>

namespace cdpr_hw
{
struct ActCoeff
{
    double act2pos, act2vel, act2effort, pos2act, vel2act, effort2act, max_out, act2pos_offset, act2vel_offset,
        act2effort_offset, kp2act, kd2act;  // for MIT Cheetah motor
};

struct ActData
{
    std::string name;
    std::string type;
    ros::Time stamp;
    uint64_t seq;
    bool halted = false;
    uint16_t q_raw;
    int16_t qd_raw;
    uint8_t temp;
    int64_t q_circle;
    uint16_t q_last;
    double frequency;
    double pos, vel, effort;
    double cmd_pos, cmd_vel, cmd_effort, exe_effort;
    double offset;
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