/**
 * @File Name: cable_archor.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2024-01-30
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#include "cdpr_cable_archor/cable_archor.hpp"

#include <std_msgs/Bool.h>
#include <unistd.h>

#include <algorithm>
#include <vector>

using namespace cable_archor;

void BaseDriver::setVel(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel)
{
    cmd.ID = 0x004 | (driver_id << 4);  // 速度模式下的参数指令，非广播
    cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
    cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
    cmd.Data[2] = static_cast<unsigned char>((target_vel >> 8) & 0xff);
    cmd.Data[3] = static_cast<unsigned char>(target_vel & 0xff);
}

void BaseDriver::setVelPos(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel, const int& target_pos)
{
    cmd.ID = 0x006 | (driver_id << 4);  // 速度位置模式下的参数指令，非广播
    cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
    cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
    cmd.Data[2] = static_cast<unsigned char>((target_vel >> 8) & 0xff);
    cmd.Data[3] = static_cast<unsigned char>(target_vel & 0xff);
    cmd.Data[4] = static_cast<unsigned char>((target_pos >> 24) & 0xff);
    cmd.Data[5] = static_cast<unsigned char>((target_pos >> 16) & 0xff);
    cmd.Data[6] = static_cast<unsigned char>((target_pos >> 8) & 0xff);
    cmd.Data[7] = static_cast<unsigned char>(target_pos & 0xff);
}

void BaseDriver::sendCmd(CanCmd& cmd_struct)
{
    if (VCI_Transmit(VCI_USBCAN2, cmd_struct.dev_ind, cmd_struct.can_ind, &cmd_struct.cmd, 1) <= 0)
    {
        ROS_ERROR("USBCAN%d CAN%d failed to send command!", cmd_struct.dev_ind, cmd_struct.can_ind);
    }
}

ArchorDriver::ArchorDriver(ros::NodeHandle& nh)
{
    nh_ = nh;
    XmlRpc::XmlRpcValue cmd_config;
    if (!(nh_.getParam("archor_config/reduction_ratio", reduction_ratio_) &&
          nh_.getParam("archor_config/encoder_lines_num", encoder_lines_num_) &&
          nh_.getParam("archor_config/cmd_config", cmd_config)))
        ROS_ERROR("Some motor params are not given in namespace: '%s/archor_config')", nh_.getNamespace().c_str());

    ROS_ASSERT(cmd_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = cmd_config.begin(); iter != cmd_config.end(); ++iter)
    {
        ROS_ASSERT(iter->second.hasMember("dev_ind") && iter->second.hasMember("can_ind") &&
                   iter->second.hasMember("driver_id") && iter->second.hasMember("direction"));

        CanCmd pub_cmd{};
        pub_cmd.dev_ind = static_cast<unsigned int>((int)iter->second["dev_ind"]);
        pub_cmd.can_ind = static_cast<unsigned int>((int)iter->second["can_ind"]);
        pub_cmd.cmd.SendType = 0;    // automatically retransmit after a failed send
        pub_cmd.cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(ArchorData{ .is_reset_ = false,
                                          .driver_id_ = static_cast<int>(iter->second["driver_id"]),
                                          .direction_ = static_cast<int>(iter->second["direction"]),
                                          .target_pos_ = 0.0,
                                          .last_pos_ = 0.0,
                                          .pub_cmd_ = std::move(pub_cmd) });
    }

    pub_ = nh_.advertise<std_msgs::Bool>("/movable_archor/ready_state", 1);
    sub_ = nh_.subscribe("/archor_coor_z", 1, &ArchorDriver::cmdPosCallback, this, ros::TransportHints().tcpNoDelay());
    ros::Duration(1.0).sleep();  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void ArchorDriver::cmdPosCallback(const cdpr_bringup::TrajCmd::ConstPtr& pos)
{
    is_traj_end_ = pos->is_traj_end;
    for (size_t i = 0; i < motor_data_.size(); ++i)
    {
        motor_data_[i].last_pos_ = motor_data_[i].target_pos_;
        motor_data_[i].target_pos_ = -pos->target[i] * motor_data_[i].direction_;
    }
}

void ArchorDriver::init(RunMode mode, const int& period)
{
    for (auto& motor_data : motor_data_)
    {
        // 发送复位指令
        motor_data.pub_cmd_.cmd.ID =
            0x000 | (motor_data.driver_id_ << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        for (auto& data : motor_data.pub_cmd_.cmd.Data)
            data = 0x55;
        sendCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd.ID = 0x001 | (motor_data.driver_id_ << 4);   // 模式选择指令
        motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>(mode);  // 选择mode对应模式
        sendCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd.Data[0] = 0x01;    // 以 1 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd.Data[1] = period;  // 以 period 毫秒为周期对外发送CTL1/CTL2的电平状态
        sendCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
}

/**
 * @brief 创建发送线程，属性设为可分离
 */
void ArchorDriver::creatThread()
{
    pthread_t thread_id;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    int ret_trans = pthread_create(&thread_id, &attr, threadFunc, this);
    ROS_ASSERT_MSG(ret_trans == 0, "Failed to create a sending thread of ArchorDriver!");
}

/**
 * @brief 发送线程函数
 * @param  arg
 * @return void*
 */
void* ArchorDriver::threadFunc(void* arg)
{
    ArchorDriver* driver = (ArchorDriver*)arg;
    driver->run();

    ROS_INFO_STREAM("Sending thread of ArchorDriver is closed!");
    pthread_exit(0);
}

/**
 * @brief 按指定模式运行电机
 */
void ArchorDriver::run()
{
    init(RunMode::VEL, 0x05);

    int run_mode = nh_.param("archor_config/run_mode", 0);
    switch (run_mode)
    {
        // reset
        case 0: {
            int target_vel = nh_.param("archor_config/target_vel", 1000);  // 速度限制值(RPM)：0~32767

            for (auto& motor_data : motor_data_)
            {
                setVel(motor_data.pub_cmd_.cmd, motor_data.driver_id_, target_vel * motor_data.direction_);
                sendCmd(motor_data.pub_cmd_);
            }

            ROS_INFO("Anchors reseting...");
            // Reset: move to bottom
            while (ros::ok())
            {
                for (auto& motor_data : motor_data_)
                {
                    if (true == motor_data.is_reset_)
                    {
                        for (int i = 0; i < 4; ++i)
                            motor_data.pub_cmd_.cmd.Data[i] = 0;
                        sendCmd(motor_data.pub_cmd_);
                    }
                }

                if (std::all_of(motor_data_.begin(), motor_data_.end(), [](const ArchorData& motor_data) {
                        return motor_data.is_reset_;
                    }))  // all archores reset successfully
                    break;
            }

            break;
        }
        // 轨迹模式
        case 1: {
            int target_vel = nh_.param("archor_config/target_vel", 1000);  // 速度限制值(RPM)：0~32767
            for (auto& motor_data : motor_data_)
            {
                setVel(motor_data.pub_cmd_.cmd, motor_data.driver_id_, target_vel * motor_data.direction_);
                sendCmd(motor_data.pub_cmd_);
            }

            ROS_INFO("Anchors reseting...");
            // Reset: move to bottom
            while (ros::ok())
            {
                for (auto& motor_data : motor_data_)
                {
                    if (true == motor_data.is_reset_)
                    {
                        for (int i = 0; i < 4; ++i)
                            motor_data.pub_cmd_.cmd.Data[i] = 0;
                        sendCmd(motor_data.pub_cmd_);
                    }
                }

                if (std::all_of(motor_data_.begin(), motor_data_.end(), [](const ArchorData& motor_data) {
                        return motor_data.is_reset_;
                    }))  // all archores reset successfully
                    break;
            }

            int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767
            init(RunMode::VEL_POS, 0x00);
            if (!(nh_.getParam("/traj/traj_period", traj_period_) && nh_.getParam("archor_config/lead", lead_)))
                ROS_ERROR("Undefined parameter: traj_period, lead!");

            std_msgs::Bool is_ready{};
            is_ready.data = true;
            pub_.publish(is_ready);
            ROS_INFO("Movable archor reset done, ready to follow the trajectory!");

            // follow the trajectory
            while (ros::ok())
            {
                for (auto& motor_data : motor_data_)
                {
                    cmd_pos =
                        motor_data.target_pos_ * 1000 * reduction_ratio_ * encoder_lines_num_ / lead_;  // m转换为qc
                    cmd_vel = std::fabs((motor_data.target_pos_ - motor_data.last_pos_) * 60 * 1000 * reduction_ratio_ /
                                        (traj_period_ * lead_));

                    setVelPos(motor_data.pub_cmd_.cmd, motor_data.driver_id_, cmd_vel, cmd_pos);
                    sendCmd(motor_data.pub_cmd_);
                }

                if (is_traj_end_)
                    break;
            }

            ros::Duration(2.0).sleep();  // buffering time for archors moving back to zero position
            break;
        }
        // 靠近电机
        case 2: {
            std::string is_stall{};
            short target_vel = nh_.param("archor_config/target_vel", 1000);  // 速度限制值(RPM)：-32768 ~ +32767
            for (auto& motor_data : motor_data_)
            {
                setVel(motor_data.pub_cmd_.cmd, motor_data.driver_id_, target_vel * motor_data.direction_);
                sendCmd(motor_data.pub_cmd_);
            }
            ROS_INFO("Press p to stop moving: ");
            while (ros::ok())
            {
                getline(std::cin, is_stall);
                if (is_stall == "p")
                    break;
            }

            for (int i = 0; i < 5; ++i)
            {
                for (auto& motor_data : motor_data_)
                {
                    for (int i = 0; i < 4; ++i)
                        motor_data.pub_cmd_.cmd.Data[i] = 0;
                    sendCmd(motor_data.pub_cmd_);
                }
            }
            break;
        }
        // 远离电机
        case 3: {
            std::string is_stall{};
            short target_vel = -nh_.param("archor_config/target_vel", 1000);  // 速度限制值(RPM)：-32768 ~ +32767

            for (auto& motor_data : motor_data_)
            {
                setVel(motor_data.pub_cmd_.cmd, motor_data.driver_id_, target_vel * motor_data.direction_);
                sendCmd(motor_data.pub_cmd_);
            }

            ROS_INFO("Press p to stop moving: ");
            while (ros::ok())
            {
                getline(std::cin, is_stall);
                if (is_stall == "p")
                    break;
            }

            for (int i = 0; i < 5; ++i)
            {
                for (auto& motor_data : motor_data_)
                {
                    for (int i = 0; i < 4; ++i)
                        motor_data.pub_cmd_.cmd.Data[i] = 0;
                    sendCmd(motor_data.pub_cmd_);
                }
            }
            break;
        }
        default:
            break;
    }
}

CableDriver::CableDriver(ros::NodeHandle& nh)
{
    nh_ = nh;
    XmlRpc::XmlRpcValue cmd_config;
    if (!(nh_.getParam("cable_config/reduction_ratio", reduction_ratio_) &&
          nh_.getParam("cable_config/encoder_lines_num", encoder_lines_num_) &&
          nh_.getParam("cable_config/cmd_config", cmd_config) &&
          nh_.getParam("cable_config/reel_diameter", reel_diameter_)))
        ROS_ERROR("Some motor params are not given in namespace: '%s/cable_config')", nh_.getNamespace().c_str());

    ROS_ASSERT(cmd_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = cmd_config.begin(); iter != cmd_config.end(); ++iter)
    {
        ROS_ASSERT(iter->second.hasMember("dev_ind") && iter->second.hasMember("can_ind") &&
                   iter->second.hasMember("driver_id") && iter->second.hasMember("direction"));

        CanCmd pub_cmd{};
        pub_cmd.dev_ind = static_cast<unsigned int>((int)iter->second["dev_ind"]);
        pub_cmd.can_ind = static_cast<unsigned int>((int)iter->second["can_ind"]);
        pub_cmd.cmd.SendType = 0;    // automatically retransmit after a failed send
        pub_cmd.cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(CableData{ .driver_id_ = static_cast<int>(iter->second["driver_id"]),
                                         .direction_ = static_cast<int>(iter->second["direction"]),
                                         .target_pos_ = 0.0,
                                         .last_pos_ = 0.0,
                                         .pub_cmd_ = std::move(pub_cmd) });
    }

    pub_ = nh_.advertise<std_msgs::Bool>("/maxon_re35/ready_state", 1);
    sub_ = nh_.subscribe("/cable_length", 1, &CableDriver::cmdCableLengthCB, this, ros::TransportHints().tcpNoDelay());
    ros::Duration(1.0).sleep();  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void CableDriver::cmdCableLengthCB(const cdpr_bringup::TrajCmd::ConstPtr& length)
{
    ROS_INFO("Current time is: %16f", ros::Time::now().toSec());  // 打印，%16f表示的是16位宽度的float类型的数字;

    is_traj_end_ = length->is_traj_end;
    for (size_t i = 0; i < motor_data_.size(); ++i)
    {
        motor_data_[i].last_pos_ = motor_data_[i].target_pos_;
        motor_data_[i].target_pos_ = length->target[i] * motor_data_[i].direction_;
    }
}

void CableDriver::init(const int& run_mode)
{
    for (auto& motor_data : motor_data_)
    {
        // 发送复位指令
        motor_data.pub_cmd_.cmd.ID =
            0x000 | (motor_data.driver_id_ << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        for (auto& data : motor_data.pub_cmd_.cmd.Data)
            data = 0x55;
        sendCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd.ID = 0x001 | (motor_data.driver_id_ << 4);  // 模式选择指令
        motor_data.pub_cmd_.cmd.Data[1] = 0x55;
        switch (run_mode)
        {
            case 0:
            case 1:
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>(RunMode::VEL_POS);  // 选择速度位置模式
                break;
            case 2:
            case 3:
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>(RunMode::VEL);  // 选择速度模式
                break;
            default:
                ROS_ERROR("Undefined run mode!");
                break;
        }
        sendCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd.Data[0] = 0x01;  // 以 1 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd.Data[1] = 0x00;
        sendCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
}

/**
 * @brief 创建线程，属性设为可分离
 */
void CableDriver::creatThread()
{
    pthread_t thread_id;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    int ret_trans = pthread_create(&thread_id, &attr, threadFunc, this);
    ROS_ASSERT_MSG(ret_trans == 0, "Failed to create a sending thread of CableDriver!");
}

/**
 * @brief 线程函数
 * @param  arg
 * @return void*
 */
void* CableDriver::threadFunc(void* arg)
{
    CableDriver* driver = (CableDriver*)arg;
    driver->run();

    ROS_INFO_STREAM("Sending thread of CableDriver is closed!");
    pthread_exit(0);
}

/**
 * @brief 按指定模式运行电机
 */
void CableDriver::run()
{
    int run_mode = nh_.param("cable_config/run_mode", 1);
    init(run_mode);

    switch (run_mode)
    {
        // 回零位
        case 0: {
            int cmd_pos = 0, cmd_vel = nh_.param("cable_config/target_vel", 1000);  // 速度限制值(RPM)：0~32767

            for (auto& motor_data : motor_data_)
            {
                setVelPos(motor_data.pub_cmd_.cmd, motor_data.driver_id_, cmd_vel, cmd_pos);
                sendCmd(motor_data.pub_cmd_);
            }

            break;
        }
        // 轨迹模式
        case 1: {
            int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767
            if (!(nh_.getParam("/traj/traj_period", traj_period_)))
                ROS_ERROR("Some motor params are not given in namespace: 'traj'");

            std_msgs::Bool is_ready{};
            is_ready.data = true;
            pub_.publish(is_ready);
            ROS_INFO("Motor RE35 Reset done, ready to follow the trajectory!");

            // follow the trajectory
            while (ros::ok())
            {
                for (auto& motor_data : motor_data_)
                {
                    cmd_pos = std::round(motor_data.target_pos_ * reduction_ratio_ * encoder_lines_num_ /
                                         (M_PI * reel_diameter_));  // m转换为qc
                    cmd_vel = std::ceil(std::fabs((motor_data.target_pos_ - motor_data.last_pos_) * 60 *
                                                  reduction_ratio_ / (traj_period_ * M_PI * reel_diameter_)));

                    setVelPos(motor_data.pub_cmd_.cmd, motor_data.driver_id_, cmd_vel, cmd_pos);
                    sendCmd(motor_data.pub_cmd_);
                }
                if (is_traj_end_)
                    break;
            }

            ros::Duration(2.0).sleep();  // buffering time for motors moving back to zero position
            break;
        }
        // 收绳: 按下按键 p
        case 2: {
            short cmd_vel = 0, target_vel = nh_.param("cable_config/target_vel",
                                                      1000);  // 速度限制值(RPM)：-32768 ~ +32767
            std::vector<int> direction{ 1, 1, 1, -1 };

            ROS_INFO("Press 'p' to move, others to stop: ");
            while (ros::ok())
            {
                if ('p' == scanKeyboard())
                    cmd_vel = target_vel;
                else
                    cmd_vel = 0;

                for (size_t i = 0; i < motor_data_.size(); ++i)
                {
                    cmd_vel *= direction[i];
                    setVel(motor_data_[i].pub_cmd_.cmd, motor_data_[i].driver_id_, cmd_vel);
                    sendCmd(motor_data_[i].pub_cmd_);
                }
            }

            break;
        }
        // 放绳: 按下按键 p
        case 3: {
            short cmd_vel = 0, target_vel = -nh_.param("cable_config/target_vel",
                                                       1000);  // 速度限制值(RPM)：-32768 ~ +32767
            std::vector<int> direction{ 1, 1, 1, -1 };

            ROS_INFO("Press 'p' to move, others to stop: ");
            while (ros::ok())
            {
                if ('p' == scanKeyboard())
                    cmd_vel = target_vel;
                else
                    cmd_vel = 0;

                for (size_t i = 0; i < motor_data_.size(); ++i)
                {
                    cmd_vel *= direction[i];
                    setVel(motor_data_[i].pub_cmd_.cmd, motor_data_[i].driver_id_, cmd_vel);
                    sendCmd(motor_data_[i].pub_cmd_);
                }
            }

            break;
        }
        default:
            break;
    }
}

int CableDriver::scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    in = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    return in;
}

UsbCan::UsbCan(ros::NodeHandle nh) : nh_(nh)
{
    archor_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor0_cur_pos", 1));
    archor_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor1_cur_pos", 1));
    archor_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor2_cur_pos", 1));
    archor_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor3_cur_pos", 1));
    cable_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable0_cur_pos", 1));
    cable_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable1_cur_pos", 1));
    cable_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable2_cur_pos", 1));
    cable_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable3_cur_pos", 1));

    // 打开设备：注意一个设备只能打开一次
    if (VCI_OpenDevice(VCI_USBCAN2, DEV_IND0, 0) != 1 || VCI_OpenDevice(VCI_USBCAN2, DEV_IND1, 0) != 1)
    {
        ROS_WARN("Failed to open at least one USBCAN!");
    }
    initCAN(VCI_USBCAN2, DEV_IND0, CAN_IND0, MotorType::MAXON_RE35);  // open USBCAN0 CNA1
    initCAN(VCI_USBCAN2, DEV_IND0, CAN_IND1, MotorType::MAXON_RE35);  // open USBCAN0 CNA2
    initCAN(VCI_USBCAN2, DEV_IND1, CAN_IND0, MotorType::MAXON_RE35);  // open USBCAN1 CNA1
    initCAN(VCI_USBCAN2, DEV_IND1, CAN_IND1, MotorType::MAXON_RE35);  // open USBCAN1 CNA2
}

UsbCan::~UsbCan()
{
    VCI_CloseDevice(VCI_USBCAN2, DEV_IND0);
    VCI_CloseDevice(VCI_USBCAN2, DEV_IND1);
}

/**
 * @brief 根据电机类型设置CAN通讯配置参数
 * @param  m_type 电机类型
 */
void UsbCan::setCANParam(const MotorType& m_type)
{
    if (m_type == MotorType::STEPPER_MOTOR)
    {
        config_.Timing0 = 0x03;  // 波特率125K（由CAN驱动器确定）
        config_.Timing1 = 0x1C;
    }
    else if (m_type == MotorType::MAXON_RE35)
    {
        config_.Timing0 = 0x00;  // 波特率1M（由RoboModule驱动器确定）
        config_.Timing1 = 0x14;
    }
    config_.AccCode = 0;
    config_.AccMask = 0xFFFFFFFF;  // 推荐为0xFFFFFFFF，即全部接收
    config_.Filter = 2;            // 接收标准帧
    config_.Mode = 0;              // 正常模式
}

/**
 * @brief 打开设备，初始化CAN并启动
 * @param  dev_type 设备类型
 * @param  dev_ind 设备索引
 * @param  can_ind can通道索引
 * @param  m_type 电机类型
 */
void UsbCan::initCAN(const int& dev_type, const int& dev_ind, const int& can_ind, const MotorType& m_type)
{
    // 配置CAN
    setCANParam(m_type);
    // 初始化CAN
    if (VCI_InitCAN(dev_type, dev_ind, can_ind, &config_) != 1)
    {
        VCI_CloseDevice(dev_type, dev_ind);
        ROS_WARN("Failed to initialize USBCAN%d CAN%d!", dev_ind, can_ind);
        return;
    }
    VCI_ClearBuffer(dev_type, dev_ind, can_ind);
    // 启动CAN
    if (VCI_StartCAN(dev_type, dev_ind, can_ind) != 1)
    {
        VCI_CloseDevice(dev_type, dev_ind);
        ROS_WARN("Failed to open USBCAN%d CAN%d!", dev_ind, can_ind);
        return;
    }
    ROS_INFO("Initialize USBCAN%d CAN%d successfully!", dev_ind, can_ind);
}

/**
 * @brief 接收电机反馈数据
 */
void UsbCan::can_receive(CableDriver& cable_driver, ArchorDriver& archor_driver)
{
    int recv_len = 0;  // 接收到的消息长度

    for (uint32_t dev_ind = 0; dev_ind < 2; ++dev_ind)
        for (uint32_t can_ind = 0; can_ind < 2; ++can_ind)
        {
            if ((recv_len = VCI_Receive(VCI_USBCAN2, dev_ind, can_ind, recv_msgs_.begin(), 3000, 100)) > 0)
            {
                for (size_t j = 0; j < recv_len; ++j)
                {
                    int id1 = recv_msgs_[j].ID - 0x0b;
                    int id2 = recv_msgs_[j].ID - 0x0c;

                    for (auto& motor_data : archor_driver.motor_data_)
                    {
                        if (id1 == motor_data.driver_id_ << 4)
                        {
                            // 接收到电流、速度、位置等信息
                            // short real_velocity = (recv_msgs_[j].Data[2] << 8) | recv_msgs_[j].Data[3];
                            int real_position = (recv_msgs_[j].Data[4] << 24) | (recv_msgs_[j].Data[5] << 16) |
                                                (recv_msgs_[j].Data[6] << 8) | recv_msgs_[j].Data[7];

                            archor_pos_.data =
                                (double)real_position * archor_driver.lead_ /
                                (1000 * archor_driver.reduction_ratio_ * archor_driver.encoder_lines_num_);
                            archor_pos_pubs_[motor_data.driver_id_ - 5].publish(archor_pos_);

                            break;
                        }
                        else if (id2 == (motor_data.driver_id_ << 4) && recv_msgs_[j].Data[0] == 0x01)
                        {
                            // 接收到锚点座驱动器CTL1/CTL2的电平状态
                            motor_data.is_reset_ = true;
                            break;
                        }
                    }
                    for (auto& motor_data : cable_driver.motor_data_)
                    {
                        if (id1 == motor_data.driver_id_ << 4)
                        {
                            // 接收到电流、速度、位置等信息
                            // short real_velocity = (recv_msgs_[j].Data[2] << 8) | recv_msgs_[j].Data[3];
                            int real_position = (recv_msgs_[j].Data[4] << 24) | (recv_msgs_[j].Data[5] << 16) |
                                                (recv_msgs_[j].Data[6] << 8) | recv_msgs_[j].Data[7];

                            cable_pos_.data = cable_driver.motor_data_[motor_data.driver_id_ - 1].direction_ *
                                              real_position * M_PI * cable_driver.reel_diameter_ /
                                              (cable_driver.reduction_ratio_ * cable_driver.encoder_lines_num_);
                            cable_pos_pubs_[motor_data.driver_id_ - 1].publish(cable_pos_);

                            break;
                        }
                    }

                    // printMsg(dev_ind, can_ind, recv_msgs_[j]);
                }
            }
            memset(&recv_msgs_, 0, sizeof(recv_msgs_));
        }
}

/**
 * @brief 按固定格式打印收到的消息
 * @param  msg
 */
void UsbCan::printMsg(uint32_t dev_ind, uint32_t can_ind, const VCI_CAN_OBJ& msg) const
{
    printf("DEV%d CAN%d RX ID:0x%08X", dev_ind, can_ind, msg.ID);  // ID
    if (msg.ExternFlag == 0)
        printf(" Standard ");  // 帧格式：标准帧
    else if (msg.ExternFlag == 1)
        printf(" Extend   ");  // 帧格式：扩展帧
    if (msg.RemoteFlag == 0)
        printf(" Data   ");  // 帧类型：数据帧
    else if (msg.RemoteFlag == 1)
        printf(" Remote ");             // 帧类型：远程帧
    printf("DLC:0x%02X", msg.DataLen);  // 帧长度
    printf(" data:0x");                 // 数据
    for (size_t i = 0; i < msg.DataLen; ++i)
        printf(" %02X", msg.Data[i]);
    printf(" TimeStamp:0x%08X", msg.TimeStamp);  // 时间戳
    printf("\n");
}