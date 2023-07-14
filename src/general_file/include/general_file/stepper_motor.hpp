/**
 * @File Name: stepper_motor.hpp
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

// 命令码(功能码)类型
constexpr int CMD_BROADCAST = 0x00;  // 广播命令(不需要返回)
constexpr int CMD_REQUEST = 0x01;    // 请求命令(需要返回)
constexpr int CMD_ACK = 0x02;        // 请求命令的正确返回
constexpr int CMD_NAK = 0x03;        // 请求命令的不正确返回
constexpr int CMD_NOCMD = 0x05;      // 无此命令码
constexpr int CMD_ER_PARA = 0x06;    // 命令参数错误

// 步进电机运行模式
enum class StepperMotorRunMode
{
    RESET,                // 复位模式
    POS,                  // 定位模式
    VEL_FORWARD,          // 正转，速度模式
    VEL_REVERSE,          // 反转，速度模式
    STALL,                // 停转
    PARAM2MEMORY = 0x13,  // 运行参数设置到内存及读取内存
    PARAM2FLASH = 0x14    // 运行参数从内存存储到 flash
};