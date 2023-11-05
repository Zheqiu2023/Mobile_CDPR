/**
 * @File Name: usb_can.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-26
 *
 *  ***********************************************************************************
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 *  ***********************************************************************************
 */
#pragma once

#include "cdpr_bringup/usb_can/controlcan.h"

#include <ros/ros.h>

enum class MotorType
{
    STEPPER_MOTOR,
    MOTOR_RE35
};

namespace can_init
{
class CanInit
{
  public:
    void initCAN(const int& DevType, const int& DevInd, const int& CANIndex, const MotorType& m_type);
    void setCANParam(const MotorType& m_type);

  private:
    VCI_INIT_CONFIG config_;  // 初始化参数，参考二次开发函数库说明书
};

/**
 * @brief 设置CAN通讯配置参数：CAN1接步进电机，CAN2接RE35和张力传感器
 * @param  m_type 电机类型
 */
void CanInit::setCANParam(const MotorType& m_type)
{
    if (m_type == MotorType::STEPPER_MOTOR)
    {
        config_.Timing0 = 0x03;  // 波特率125K（由CAN驱动器确定）
        config_.Timing1 = 0x1C;
    }
    else if (m_type == MotorType::MOTOR_RE35)
    {
        config_.Timing0 = 0x00;  // 波特率1M（由RoboModule驱动器确定）
        config_.Timing1 = 0x14;
    }
    config_.AccCode = 0;
    config_.AccMask = 0xFFFFFFFF;  // 推荐为0xFFFFFFFF，即全部接收
    config_.Filter = 2;            // 接收标准帧
    config_.Mode = 0;              // 正常模式
}

/**
 * @brief 打开设备，初始化CAN并启动
 * @param  DevType 设备类型
 * @param  DevInd 设备索引
 * @param  CANIndex can通道索引
 * @param  m_type 电机类型
 * @return true
 * @return false
 */
void CanInit::initCAN(const int& DevType, const int& DevInd, const int& CANIndex, const MotorType& m_type)
{
    // 配置CAN
    setCANParam(m_type);
    // 初始化CAN
    if (VCI_InitCAN(DevType, DevInd, CANIndex, &config_) != 1)
    {
        VCI_CloseDevice(DevType, DevInd);
        if (CANIndex == CAN_IND0)
            ROS_ERROR_STREAM("Failed to initialize CAN1!");
        else
            ROS_ERROR_STREAM("Failed to initialize CAN2!");
        return;
    }
    VCI_ClearBuffer(DevType, DevInd, CANIndex);
    // 启动CAN
    if (VCI_StartCAN(DevType, DevInd, CANIndex) != 1)
    {
        VCI_CloseDevice(DevType, DevInd);
        if (CANIndex == CAN_IND0)
            ROS_ERROR_STREAM("Failed to open CAN1!");
        else
            ROS_ERROR_STREAM("Failed to open CAN2!");
        return;
    }
    ROS_INFO_STREAM("Initialize CAN successfully!");
}
}  // namespace can_init