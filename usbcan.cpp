#include "usbcan.hpp"

#include <QDebug>
#include <QTime>
#include <QDir>

using namespace usb_can;

UsbCan::UsbCan(QObject* parent)
{
    // 打开设备：注意一个设备只能打开一次
    if (VCI_OpenDevice(VCI_USBCAN2, DEV_IND0, 0) != 1 || VCI_OpenDevice(VCI_USBCAN2, DEV_IND1, 0) != 1)
    {
        qWarning() << "Failed to open at least one USBCAN!";
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
        qWarning() << "Failed to initialize USBCAN" << dev_ind << " CAN" << can_ind << "!";
        return;
    }
    VCI_ClearBuffer(dev_type, dev_ind, can_ind);
    // 启动CAN
    if (VCI_StartCAN(dev_type, dev_ind, can_ind) != 1)
    {
        VCI_CloseDevice(dev_type, dev_ind);
        qWarning() << "Failed to open USBCAN" << dev_ind << " CAN" << can_ind << "!";
        return;
    }
    qInfo() << "Initialize USBCAN" << dev_ind << " CAN" << can_ind << " successfully!";
}

void UsbCan::stopRun()
{
    stop_run_ = true;
}

/**
 * @brief 接收电机反馈数据
 */
void UsbCan::recvPos()
{
    int recv_len = 0;  // 接收到的消息长度
    qint64 cur_time = 0;
    QString message{};
    // path to save .csv file
    QString dir_path = QCoreApplication::applicationDirPath() + "/../data";
    // check if file directory exists
    QDir dir;
    if (!dir.exists(dir_path))
        dir.mkpath(dir_path);
    // .csv file path
    QString file_path =
        dir_path + "/" + QString("recvPos%1.csv").arg(QDateTime::currentDateTime().toString("MM_dd_hh_mm_ss"));
    // create and open .csv file
    QFile file(file_path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    stop_run_ = false;
    while (!stop_run_)
    {
        for (uint32_t dev_ind = 0; dev_ind < 2; ++dev_ind)
            for (uint32_t can_ind = 0; can_ind < 2; ++can_ind)
            {
                if ((recv_len = VCI_Receive(VCI_USBCAN2, dev_ind, can_ind, recv_msgs_.begin(), 3000, 100)) > 0)
                {
                    for (int j = 0; j < recv_len; ++j)
                    {
                        int id1 = recv_msgs_[j].ID - 0x0b;
                        for (size_t i = 0; i < 4; ++i)
                        {
                            if (id1 == params_.archor_id_[i] << 4)
                            {
                                // 接收到锚点座驱动器电流、速度、位置等信息
                                // short real_velocity = (recv_msgs_[j].Data[2] << 8) | recv_msgs_[j].Data[3];
                                int real_position = (recv_msgs_[j].Data[4] << 24) | (recv_msgs_[j].Data[5] << 16) |
                                                    (recv_msgs_[j].Data[6] << 8) | recv_msgs_[j].Data[7];

                                archor_pos_ = (double)real_position * params_.lead_ /
                                              (1000 * params_.reduction_ratio_ * params_.encoder_lines_num_);
                                cur_time = QDateTime::currentMSecsSinceEpoch();
                                message =
                                    QString(u8"%1, %2, %3").arg(cur_time).arg(params_.archor_id_[i]).arg(archor_pos_);
                                out << message << '\n';
                                break;
                            }
                            else if (id1 == params_.cable_id_[i] << 4)
                            {
                                // 接收到绳索驱动器电流、速度、位置等信息
                                // short real_velocity = (recv_msgs_[j].Data[2] << 8) | recv_msgs_[j].Data[3];
                                int real_position = (recv_msgs_[j].Data[4] << 24) | (recv_msgs_[j].Data[5] << 16) |
                                                    (recv_msgs_[j].Data[6] << 8) | recv_msgs_[j].Data[7];

                                cable_pos_ = (double)real_position * params_.cable_direction_[i] * M_PI *
                                             params_.reel_diameter_ /
                                             (params_.reduction_ratio_ * params_.encoder_lines_num_);
                                cur_time = QDateTime::currentMSecsSinceEpoch();
                                message =
                                    QString(u8"%1, %2, %3").arg(cur_time).arg(params_.cable_id_[i]).arg(cable_pos_);
                                out << message << '\n';
                                break;
                            }
                        }
                        // printMsg(dev_ind, can_ind, recv_msgs_[j]);
                    }
                }
                memset(&recv_msgs_, 0, sizeof(recv_msgs_));
            }
    }
    file.close();
}

/**
 * @brief 按固定格式打印收到的消息
 * @param  msg
 */
void UsbCan::printMsg(unsigned int dev_ind, unsigned int can_ind, const VCI_CAN_OBJ& msg) const
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
