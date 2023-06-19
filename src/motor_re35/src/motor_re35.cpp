/**
 * @File Name: motor_re35.cpp
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
#include "motor_re35.hpp"

#include <pthread.h>
#include <ros/ros.h>
#include <stdio.h>

#include <array>
#include <vector>

#include "usb_can.hpp"

using namespace motor_re35;

Re35Run::Re35Run()
{
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

/**
 * @brief 接收线程
 * @param  arg
 * @return void*
 */
void* Re35Run::receiveFunc(void* arg)
{
    int16_t reclen = 0;
    std::array<VCI_CAN_OBJ, 3000> rec_msgs{};  // 接收缓存，设为3000为佳
    Re35Run* re35_run = (Re35Run*)arg;

    while (ros::ok())
    {
        // 调用接收函数，如果有数据，进行数据处理显示。
        if ((reclen = VCI_Receive(VCI_USBCAN2, DEV_IND, CAN_IND1, rec_msgs.begin(), 3000, 100)) > 0)
        {
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
                {
                    printf(" %02X", rec_msgs[j].Data[i]);
                }
                printf(" TimeStamp:0x%08X\n", rec_msgs[j].TimeStamp);  // 时间标识
                printf(" Actual current value: %hd", (rec_msgs[j].Data[0] << 8) | rec_msgs[j].Data[1]);
                printf(" Actual velocity value: %hd", (rec_msgs[j].Data[2] << 8) | rec_msgs[j].Data[3]);
                printf(" Actual position value: %hd\n", (rec_msgs[j].Data[4] << 24) | (rec_msgs[j].Data[5] << 16) |
                                                            (rec_msgs[j].Data[6] << 8) | rec_msgs[j].Data[7]);
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
void* Re35Run::transmitFunc(void* arg)
{
    Re35Run* re35_run = (Re35Run*)arg;

    // 发送复位指令
    re35_run->sender_.setCmd(Re35RunMode::RESET);
    re35_run->sender_.transmitCmd();
    usleep(500000);
    // 发送配置指令
    re35_run->sender_.setCmd(Re35RunMode::CONFIG);
    re35_run->sender_.transmitCmd();
    usleep(500000);
    // 发送模式选择指令
    re35_run->sender_.setCmd(Re35RunMode::MODE_SELECTION);
    re35_run->sender_.transmitCmd();
    usleep(500000);
    // 发送数据指令
    std::vector<int> goal_pos_vec{};
    int reduction_ratio = 0, encoder_lines_num = 0, temp_pos = 0, history_pos = 0;
    ros::param::get("/re35/reduction_ratio", reduction_ratio);
    ros::param::get("/re35/encoder_lines_num", encoder_lines_num);
    ros::param::get("/re35/motor_ctrl_data/goal_pos_vec", goal_pos_vec);

    re35_run->sender_.send_cmd_.ID = 0x016;  // 速度位置模式下的参数指令，非广播
    re35_run->sender_.send_cmd_.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
    re35_run->sender_.send_cmd_.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
    re35_run->sender_.send_cmd_.Data[2] = static_cast<unsigned char>((VEL_LIM >> 8) & 0xff);
    re35_run->sender_.send_cmd_.Data[3] = static_cast<unsigned char>(VEL_LIM & 0xff);
    for (auto& temp_pos : goal_pos_vec)
    {
        history_pos += temp_pos;
        temp_pos = history_pos * reduction_ratio * encoder_lines_num / 360;  // °转换为qc
        re35_run->sender_.send_cmd_.Data[4] = static_cast<unsigned char>((temp_pos >> 24) & 0xff);
        re35_run->sender_.send_cmd_.Data[5] = static_cast<unsigned char>((temp_pos >> 16) & 0xff);
        re35_run->sender_.send_cmd_.Data[6] = static_cast<unsigned char>((temp_pos >> 8) & 0xff);
        re35_run->sender_.send_cmd_.Data[7] = static_cast<unsigned char>(temp_pos & 0xff);

        re35_run->sender_.transmitCmd();
        sleep(1);
    }
    pthread_exit(0);
}

/**
 * @brief 根据电机运行模式设置相应控制指令
 * @param  cmd_mode
 */
void Re35Sender::setCmd(Re35RunMode cmd_mode)
{
    this->send_cmd_.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
    this->send_cmd_.RemoteFlag = 0;  // 0为数据帧，1为远程帧
    this->send_cmd_.ExternFlag = 0;  // 0为标准帧，1为拓展帧
    this->send_cmd_.DataLen = 8;     // 数据长度8字节
    switch (cmd_mode)
    {
        case Re35RunMode::RESET:
            this->send_cmd_.ID = 0x000;  // 广播复位指令（帧ID，由驱动器编号和功能序号决定）
            for (auto& data : this->send_cmd_.Data)
                data = 0x55;
            break;
        case Re35RunMode::MODE_SELECTION:
            this->send_cmd_.ID = 0x001;  // 广播模式选择指令
            for (auto& data : this->send_cmd_.Data)
                data = 0x55;
            this->send_cmd_.Data[0] = 0x05;  // 位置速度模式
            break;
        case Re35RunMode::CONFIG:
            this->send_cmd_.ID = 0x00A;  // 广播配置指令
            for (auto& data : this->send_cmd_.Data)
                data = 0x55;
            this->send_cmd_.Data[0] = 0xc8;  // 以 200 毫秒为周期对外发送电流、速度、位置等信息
            this->send_cmd_.Data[1] = 0x00;
            break;
        default:
            ROS_INFO_STREAM("Motor operating mode error!");
            break;
    }
}

/**
 * @brief 发送指令
 */
inline void Re35Sender::transmitCmd()
{
    if (VCI_Transmit(VCI_USBCAN2, DEV_IND, CAN_IND1, &this->send_cmd_, 1) <= 0)
    {
        ROS_ERROR_STREAM("Failed to send command!");
        exit(EXIT_FAILURE);
    }
}
