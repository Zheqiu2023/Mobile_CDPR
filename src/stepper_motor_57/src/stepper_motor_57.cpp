/**
 * @File Name: stepper_motor_57.cpp
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

#include "stepper_motor_57.hpp"

#include <pthread.h>
#include <ros/ros.h>
#include <stdio.h>

#include <array>
#include <string>
#include <vector>

#include "usb_can.hpp"
using namespace motor_57;

Motor57Run::Motor57Run()
{
    sem_init(&this->sem_trans_, 0, 0);  // 将信号量sem_trans设为线程间通信，初值为0
    pthread_t thread_rec, thread_trans;
    int ret_rec = pthread_create(&thread_rec, nullptr, receiveFunc, this);
    ROS_ASSERT_MSG(ret_rec == 0, "Failed to create a receiving thread!");
    int ret_trans = pthread_create(&thread_trans, nullptr, transmitFunc, this);
    ROS_ASSERT_MSG(ret_trans == 0, "Failed to create a sending thread!");

    // 等待线程关闭
    pthread_join(thread_trans, nullptr);
    ROS_INFO_STREAM("Sending thread is closed!");
    pthread_cancel(thread_rec);
    ROS_INFO_STREAM("Receiving thread is closed!");
}

Motor57Run::~Motor57Run()
{
    sem_destroy(&this->sem_trans_);
}

/**
 * @brief 接收线程
 * @param  arg
 * @return void*
 */
void* Motor57Run::receiveFunc(void* arg)
{
    int reclen = 0;
    std::array<VCI_CAN_OBJ, 3000> rec_msgs{};  // 接收缓存，设为3000为佳
    Motor57Run* m57_run = (Motor57Run*)arg;

    while (ros::ok())
    {
        // 调用接收函数，如果有数据，进行数据处理显示
        if ((reclen = VCI_Receive(VCI_USBCAN2, DEV_IND, CAN_IND0, rec_msgs.begin(), 3000, 100)) > 0)
        {
            // 若复位成功，将发送信号量置1，执行定位功能
            if (rec_msgs[0].Data[2] == 0x41 && rec_msgs[0].Data[7] == 0)
                sem_post(&m57_run->sem_trans_);
            for (size_t j = 0; j < reclen; ++j)
            {
                printf("CAN%d RX ID:0x%08X", CAN_IND0 + 1, rec_msgs[j].ID);  // ID
                if (rec_msgs[j].ExternFlag == 0)
                    printf(" Standard ");                                    // 帧格式：标准帧
                if (rec_msgs[j].ExternFlag == 1)
                    printf(" Extend   ");                                    // 帧格式：扩展帧
                if (rec_msgs[j].RemoteFlag == 0)
                    printf(" Data   ");                                      // 帧类型：数据帧
                if (rec_msgs[j].RemoteFlag == 1)
                    printf(" Remote ");                                      // 帧类型：远程帧
                printf("DLC:0x%02X", rec_msgs[j].DataLen);                   // 帧长度
                printf(" data:0x");                                          // 数据
                for (size_t i = 0; i < rec_msgs[j].DataLen; ++i)
                    printf(" %02X", rec_msgs[j].Data[i]);
                printf(" TimeStamp:0x%08X", rec_msgs[j].TimeStamp);  // 时间标识
                printf("\n");
            }
        }
    }
    pthread_exit(0);
}

/**
 * @brief 发送线程函数
 * @param  arg
 * @return void*
 */
void* Motor57Run::transmitFunc(void* arg)
{
    Motor57Run* m57_run = (Motor57Run*)arg;

    // Motor57Param param;
    // param.writeParam();
    // param.readParam();
    // usleep(100000);

    std::string cmd_type{ "" };
    ros::param::get("/motor_57/motor_cmd_type", cmd_type);

    if (cmd_type == "position")
    {
        ROS_INFO_STREAM("Reset before positioning!");
        // 先复位后定位
        m57_run->sender_.setCmd(StepperMotorRunMode::RESET);
        m57_run->sender_.transmitCmd();
        sem_wait(&m57_run->sem_trans_);
        ROS_INFO_STREAM("Reset done!");
        m57_run->sender_.setCmd(StepperMotorRunMode::POS);
        m57_run->sender_.transmitCmd();
    }
    else if (cmd_type == "vel_forward")
    {
        ROS_INFO_STREAM("Forward rotation!");
        // 正转5s后停转
        m57_run->sender_.setCmd(StepperMotorRunMode::VEL_FORWARD);
        m57_run->sender_.transmitCmd();
        sleep(5);
        m57_run->sender_.setCmd(StepperMotorRunMode::STALL);
        m57_run->sender_.transmitCmd();
    }
    else if (cmd_type == "vel_backward")
    {
        ROS_INFO_STREAM("Reverse rotation!");
        // 反转5s后停转
        m57_run->sender_.setCmd(StepperMotorRunMode::VEL_REVERSE);
        m57_run->sender_.transmitCmd();
        sleep(3);
        m57_run->sender_.setCmd(StepperMotorRunMode::STALL);
        m57_run->sender_.transmitCmd();
    }
    else
    {
        ROS_WARN_STREAM("Motor control type error!");
    }
    pthread_exit(0);
}

/**
 * @brief 根据电机运行模式设置相应控制指令
 * @param  cmd_mode
 */
void Motor57Sender::setCmd(StepperMotorRunMode cmd_mode)
{
    std::vector<int> data_vec{ 0, 0, 0, 0 };

    this->send_cmd_.ID = 0xc1;  // 帧ID，与驱动器地址相同
    this->send_cmd_.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
    this->send_cmd_.RemoteFlag = 0;                                   // 0为数据帧，1为远程帧
    this->send_cmd_.ExternFlag = 0;                                   // 0为标准帧，1为拓展帧
    this->send_cmd_.DataLen = 8;                                      // 数据长度8字节
    this->send_cmd_.Data[0] = static_cast<unsigned char>(0xc1 >> 3);  // CAN 驱动器地址0x0c1(11bits)的高 8bits
    this->send_cmd_.Data[1] = static_cast<unsigned char>(0xc1 << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
    switch (cmd_mode)
    {
        case StepperMotorRunMode::RESET:
            this->send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X01);  // 复位命令(需要返回)
            this->send_cmd_.Data[7] = 1;  // 复位完毕，返回命令
            break;
        case StepperMotorRunMode::POS:
            ros::param::get("/motor_57/motor_ctrl_data/goal_pos_arr", data_vec);
            this->send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X02);  // 定位命令(需要返回)
            this->send_cmd_.Data[7] = 1;  // 到位后，返回命令
            break;
        case StepperMotorRunMode::VEL_FORWARD:
            ros::param::get("/motor_57/motor_ctrl_data/goal_vel_forward_arr", data_vec);
            this->send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X03);  // 正转命令(需要返回)
            this->send_cmd_.Data[7] = 3;  // 正方向转 IntDate 步后命令返回
            break;
        case StepperMotorRunMode::VEL_REVERSE:
            ros::param::get("/motor_57/motor_ctrl_data/goal_vel_reverse_arr", data_vec);
            this->send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X04);  // 反转命令(需要返回)
            this->send_cmd_.Data[7] = 3;  // 反方向转 IntDate 步后命令返回
            break;
        case StepperMotorRunMode::STALL:
            this->send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X05);  // 停转命令(需要返回)
            this->send_cmd_.Data[7] = 1;  // 减速停止，返回命令
            break;
        default:
            ROS_WARN_STREAM("Motor operation mode error!");
            break;
    }
    for (size_t i = 3; i < 7; ++i)
    {
        this->send_cmd_.Data[i] = static_cast<unsigned char>(data_vec[i - 3]);
    }
}

/**
 * @brief 发送函数
 */
inline void Motor57Sender::transmitCmd()
{
    if (VCI_Transmit(VCI_USBCAN2, DEV_IND, CAN_IND0, &this->send_cmd_, 1) <= 0)
    {
        ROS_ERROR_STREAM("Failed to send command!");
    }
}

/**
 * @brief 发送运行参数,保存到flash后掉电数据不会丢失，因此只需设置一次
 */
void Motor57Param::writeParam()
{
    std::array<VCI_CAN_OBJ, 7> data_arr;  // data_arr[0]:启动周期   data_arr[1]:恒速周期    data_arr[2]:加速步数
                                          // data_arr[3]:加速系数   data_arr[4]:细分    data_arr[5]：工作模式
                                          // data_arr[6]：相电流
    for (auto& data : data_arr)
    {
        data.ID = 0xc1;  // 帧ID，与驱动器地址相同
        data.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
        data.RemoteFlag = 0;                                   // 0为数据帧，1为远程帧
        data.ExternFlag = 0;                                   // 0为标准帧，1为拓展帧
        data.DataLen = 8;                                      // 数据长度8字节
        data.Data[0] = static_cast<unsigned char>(0xc1 >> 3);  // CAN 驱动器地址0x0c1(11bits)的高 8bits
        data.Data[1] = static_cast<unsigned char>(0xc1 << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
        data.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数保存到内存
    }
    // 小端模式：低地址存放低位
    std::vector<std::vector<int>> data_vec(7);
    ros::param::get("/motor_57/operating_param/plus_start_time", data_vec[0]);
    ros::param::get("/motor_57/operating_param/plus_constant_time", data_vec[1]);
    ros::param::get("/motor_57/operating_param/acc_steps", data_vec[2]);
    ros::param::get("/motor_57/operating_param/acc_cof", data_vec[3]);
    ros::param::get("/motor_57/operating_param/sub_divide", data_vec[4]);
    ros::param::get("/motor_57/operating_param/reset_mode", data_vec[5]);
    ros::param::get("/motor_57/operating_param/phase_current", data_vec[6]);
    std::cout << data_vec.size() << std::endl;
    for (size_t i = 0; i < data_vec.size(); ++i)
    {
        for (size_t j = 0; j < 5; ++j)
        {
            data_arr[i].Data[j + 3] = static_cast<unsigned char>(data_vec[i][j]);
        }
    }

    ROS_INFO_STREAM("Sending operating parameters!");
    for (auto& data : data_arr)
    {
        if (VCI_Transmit(VCI_USBCAN2, DEV_IND, CAN_IND0, &data, 1) <= 0)
        {
            ROS_WARN_STREAM("Failed to save operating parameters to memory!");
            return;
        }
        usleep(100000);
    }
    ROS_INFO_STREAM("********************************");
    data_arr[0].Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X14);  // 运行参数保存到flash，掉电数据不丢失
    if (VCI_Transmit(VCI_USBCAN2, DEV_IND, CAN_IND0, &data_arr[0], 1) <= 0)
    {
        ROS_WARN_STREAM("Failed to save operating parameters to flash!");
        return;
    }
    usleep(100000);
    ROS_INFO_STREAM("Parameters have been saved in flash!");
}

/**
 * @brief 从flash中读取电机运行参数
 */
void Motor57Param::readParam()
{
    std::array<VCI_CAN_OBJ, 7> data_arr;  // data_arr[0]:启动周期   data_arr[1]:恒速周期    data_arr[2]:加速步数
                                          // data_arr[3]:加速系数   data_arr[4]:细分    data_arr[5]：工作模式
                                          // data_arr[6]：相电流
    for (auto& data : data_arr)
    {
        data.ID = 0xc1;  // 帧ID，与驱动器地址相同
        data.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
        data.RemoteFlag = 0;                                   // 0为数据帧，1为远程帧
        data.ExternFlag = 0;                                   // 0为标准帧，1为拓展帧
        data.DataLen = 8;                                      // 数据长度8字节
        data.Data[0] = static_cast<unsigned char>(0xc1 >> 3);  // CAN 驱动器地址0x0c1(11bits)的高 8bits
        data.Data[1] = static_cast<unsigned char>(0xc1 << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
        data.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数读取
        for (size_t i = 3; i < data.DataLen; ++i)
        {
            data.Data[i] = 0;
        }
    }
    data_arr[0].Data[7] = 17;
    data_arr[1].Data[7] = 15;
    data_arr[2].Data[7] = 1;
    data_arr[3].Data[7] = 19;
    data_arr[4].Data[7] = 11;
    data_arr[5].Data[7] = 13;
    data_arr[6].Data[7] = 23;

    ROS_INFO_STREAM("Reading operating parameters!");
    for (auto& data : data_arr)
    {
        if (VCI_Transmit(VCI_USBCAN2, DEV_IND, CAN_IND0, &data, 1) <= 0)
        {
            ROS_WARN_STREAM("Failed to read operating parameters!");
            return;
        }
        usleep(100000);
    }
}
