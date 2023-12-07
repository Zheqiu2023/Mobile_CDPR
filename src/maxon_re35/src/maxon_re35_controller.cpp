/**
 * @File Name: maxon_re35_controller.cpp
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

#include "maxon_re35/maxon_re35.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "maxon_re35");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    maxon_re35::MotorDriver m_run(nh);

    m_run.run();

    ros::waitForShutdown();
    return 0;
}