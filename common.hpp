#pragma once

#include <QCoreApplication>
#include <QTime>

#include "controlcan.hpp"

class TrajParams
{
  public:
    const double reel_diameter_ = 0.03;                                     // 绕线轮直径
    const int reduction_ratio_ = 35, encoder_lines_num_ = 2000, lead_ = 5;  // re35电机减速比、编码器线数、丝杠导程
    const std::vector<int> archor_id_ = { 5, 6, 7, 8 }, archor_direction_ = { -1, -1, -1, -1 },
                           cable_id_ = { 1, 2, 3, 4 }, cable_direction_ = { -1, 1, 1, -1 };
};

namespace motor_driver
{
constexpr unsigned short PWM_LIM = 5000;  // pwm限制值
// maxon re35电机运行模式
enum class RunMode
{
    VEL = 0X03,     // 速度模式
    VEL_POS = 0X05  // 速度位置模式
};

struct CanCmd
{
    int dev_ind;  // usbcan设备索引
    int can_ind;  // can通道索引
    VCI_CAN_OBJ cmd;
};

struct CableData
{
    int driver_id_, direction_;
    double target_pos_, last_pos_;
    CanCmd pub_cmd_;
};

struct ArchorData
{
    bool is_reset_;
    int driver_id_, direction_;
    double target_pos_, last_pos_;
    CanCmd pub_cmd_;
};
}  // namespace motor_driver

namespace usb_can
{
enum class MotorType
{
    STEPPER_MOTOR,  // 步进电机
    MAXON_RE35      // re35电机
};
}
