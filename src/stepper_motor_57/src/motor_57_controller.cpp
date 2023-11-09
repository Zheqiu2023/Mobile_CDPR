/**
 * @File Name: motor_57_controller.cpp
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

// 开发库中的样例只是提供一个简单的调用so库的方法供参考，程序接收与发送函数设置在两个线程中，并且线程没有同步。
// 现实中客户编程中，发送与接收函数不能同时调用（不支持多线程），如果在多线程中，一定需要互锁。需要客户自行完善代码。

#include "stepper_motor_57/stepper_motor_57.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stepper_motor_57");
    ros::NodeHandle nh("~");

    motor_57::MotorRun m_run(nh);
    m_run.run();

    ros::spin();
    return 0;
}
