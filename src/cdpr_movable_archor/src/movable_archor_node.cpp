/**
 * @File Name: movable_archor_node.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-12-14
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#include "cdpr_movable_archor/movable_archor.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdpr_movable_archor");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    movable_archor::ArchorDriver archor_driver(nh);
    archor_driver.run();

    return 0;
}