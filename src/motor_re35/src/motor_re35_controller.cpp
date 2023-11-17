/**
 * @File Name: motor_re35_controller.cpp
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

#include "motor_re35/motor_re35.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_re35");
    ros::NodeHandle nh("~");

    motor_re35::MotorRun m_run(nh);

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        m_run.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}