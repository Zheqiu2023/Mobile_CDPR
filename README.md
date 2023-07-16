## Code Style Guide
遵循[ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

使用[代码格式自动化工具](https://github.com/PickNikRobotics/roscpp_code_format)

## Platform
运行在`ros noetic`上，修改`.vscode`中的配置文件后重新编译即可运行在自己电脑上

## TODO
1. 给串口号起别名，防止串口号变化
2. 利用rosserial实现stm32与ros的通信
3. 通过ssh、nfs挂载实现远程控制机器人
