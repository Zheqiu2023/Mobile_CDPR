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

#include <ros/ros.h>

#include "cdpr_bringup/usb_can/controlcan.h"

namespace usb_can {
enum class MotorType { STEPPER_MOTOR, MAXON_RE35 };

class CanInit {
   public:
    void initCAN(const int& dev_type, const int& dev_ind, const int& can_ind, const MotorType& m_type);

   private:
    void setCANParam(const MotorType& m_type);
    VCI_INIT_CONFIG config_;  // 初始化参数，参考二次开发函数库说明书
};

/**
 * @brief 根据电机类型设置CAN通讯配置参数
 * @param  m_type 电机类型
 */
void CanInit::setCANParam(const MotorType& m_type) {
    if (m_type == MotorType::STEPPER_MOTOR) {
        config_.Timing0 = 0x03;  // 波特率125K（由CAN驱动器确定）
        config_.Timing1 = 0x1C;
    } else if (m_type == MotorType::MAXON_RE35) {
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
 * @param  dev_type 设备类型
 * @param  dev_ind 设备索引
 * @param  can_ind can通道索引
 * @param  m_type 电机类型
 * @return true
 * @return false
 */
void CanInit::initCAN(const int& dev_type, const int& dev_ind, const int& can_ind, const MotorType& m_type) {
    // 配置CAN
    setCANParam(m_type);
    // 初始化CAN
    if (VCI_InitCAN(dev_type, dev_ind, can_ind, &config_) != 1) {
        VCI_CloseDevice(dev_type, dev_ind);
        ROS_WARN("Failed to initialize USBCAN%d CAN%d!", dev_ind, can_ind);
        return;
    }
    VCI_ClearBuffer(dev_type, dev_ind, can_ind);
    // 启动CAN
    if (VCI_StartCAN(dev_type, dev_ind, can_ind) != 1) {
        VCI_CloseDevice(dev_type, dev_ind);
        ROS_WARN("Failed to open USBCAN%d CAN%d!", dev_ind, can_ind);
        return;
    }
    ROS_INFO("Initialize USBCAN%d CAN%d successfully!", dev_ind, can_ind);
}
}  // namespace usb_can