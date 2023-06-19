/**
 * @File Name: stepper_motor.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-26
 *
 *  ***********************************************************************************
 *  BSD 3-Clause License
 *
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1.Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  2.Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  3.Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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