/**
 * @File Name: cable_archor_node.cpp
 * @brief
 * @author qz (zheqiu2021@163.com)
 * @version 0.1
 * @date 2024-01-31
 *
 * @copyright Copyright (c) 2024
 */
#include "cdpr_cable_archor/cable_archor.hpp"

using namespace cable_archor;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cdpr_cable_archor");
    ros::NodeHandle nh("~");

    // 开启1条并发线程处理订阅话题回调函数，保证及时接收到每条消息
    ros::AsyncSpinner spinner(4);
    spinner.start();

    UsbCan can(nh);
    CableDriver cable_driver(nh);
    ArchorDriver archor_driver(nh);
    cable_driver.creatThread();
    archor_driver.creatThread();

    ros::Rate loop(100);
    while (ros::ok()) {
        can.can_receive(cable_driver, archor_driver);
        loop.sleep();
    }

    return 0;
}