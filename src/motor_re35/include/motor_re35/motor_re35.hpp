/**
 * @File Name: motor_re35.hpp
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

#include "controlcan.h"

enum class Re35RunMode
{
    RESET,           // 复位
    MODE_SELECTION,  // 模式选择
    CONFIG,          // 配置指令
};

constexpr signed short PWM_LIM = 5000;  // pwm限制值
constexpr signed short VEL_LIM = 6000;  // 速度限制值(RPM)

namespace motor_re35
{
class Re35Sender
{
  public:
    void transmitCmd();                 // 发送指令
    void setCmd(Re35RunMode cmd_mode);  // 设置指令
    VCI_CAN_OBJ send_cmd_{};            // 待发送的指令
};

class Re35Run
{
  public:
    Re35Run();
    static void* receiveFunc(void* arg);
    static void* transmitFunc(void* arg);

  private:
    Re35Sender sender_{};
};
}  // namespace motor_re35
