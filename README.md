## Code Style Guide
遵循[ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

使用[代码格式自动化工具](https://github.com/PickNikRobotics/roscpp_code_format)

## Platform
运行在`ros noetic`上，修改`.vscode`中的配置文件后重新编译即可运行在自己电脑上

## 功能包说明
1. `cdpr_bringup`: 包含配置文件、通用头文件和源文件、外部库、launch文件、msg...
2. `cdpr_cable_archor`: 控制绳索和锚点座运行
3. `cdpr_chassis_controller`: 底盘控制器（弃用）
4. `cdpr_control`: 底盘运动控制，读取局部轨迹和全局轨迹
5. `cdpr_description`: cdpr 描述文件
6. `cdpr_gazebo`: gazebo仿真控制文件
7. `cdpr_hw`: cdpr控制接口（弃用）
8. `cdpr_movable_archor`: 动锚点座运动控制，使用maxon re35电机
9. `cdpr_teleop`: 键盘远程控制底盘运动
10. `cdpr_urdf`: cdpr urdf描述文件
11. `maxon_re35`: 绳索运动控制
12. `stepper_motor_57`: 动锚点座运动控制，使用57步进电机（弃用）
13. `unitree_motor_a1`: 车轮运动控制
14. `unitree_motor_go`: 车轮转向控制
15. `usb_can`: 使用USBCAN收发指令
16. `wit_ros_imu`: 维特智能imu软件包

## TODO
1. 给串口号起别名，防止串口号变化
2. 利用rosserial实现stm32与ros的通信
