#include "widget.hpp"
#include "ui_widget.h"

#include <QDebug>
#include <QSpinBox>
#include <QRadioButton>
#include <QPushButton>
#include <QMessageBox>

Widget::Widget(QWidget* parent) : QWidget(parent), ui(new Ui::Widget)
{
    ui->setupUi(this);
    // 创建子线程
    archor_thread_ = new QThread;
    cable_thread_ = new QThread;
    can_thread_ = new QThread;
    // 创建任务对象
    archors_ = new motor_driver::ArchorDriver;
    cables_ = new motor_driver::CableDriver;
    usbcan_ = new usb_can::UsbCan;
    // 移动任务对象到子线程中
    archors_->moveToThread(archor_thread_);
    cables_->moveToThread(cable_thread_);
    usbcan_->moveToThread(can_thread_);
    // 释放线程资源
    connect(archor_thread_, &QThread::finished, archors_, &QObject::deleteLater);  // 释放任务对象
    connect(archor_thread_, &QThread::finished, archor_thread_,
            &QObject::deleteLater);  // 释放子线程, 也可 archor_thread_ = new QThread(this)
    connect(cable_thread_, &QThread::finished, cables_, &QObject::deleteLater);
    connect(cable_thread_, &QThread::finished, cable_thread_, &QObject::deleteLater);
    connect(can_thread_, &QThread::finished, usbcan_, &QObject::deleteLater);
    connect(can_thread_, &QThread::finished, can_thread_, &QObject::deleteLater);
    // 连接信号与槽
    connect(this, &Widget::startArchorReset, archors_, &motor_driver::ArchorDriver::reset);
    connect(this, &Widget::startArchorRunTraj, archors_, &motor_driver::ArchorDriver::run_traj);
    connect(this, &Widget::startCableRunTraj, cables_, &motor_driver::CableDriver::run_traj);
    connect(this, &Widget::startCanRecv, usbcan_, &usb_can::UsbCan::can_receive);

    // archor button
    connect(ui->lfArchorUp, &QRadioButton::toggled, this, &Widget::on_lfArchorUp_toggled);
    connect(ui->rfArchorUp, &QRadioButton::toggled, this, &Widget::on_rfArchorUp_toggled);
    connect(ui->lbArchorUp, &QRadioButton::toggled, this, &Widget::on_lbArchorUp_toggled);
    connect(ui->rbArchorUp, &QRadioButton::toggled, this, &Widget::on_rbArchorUp_toggled);
    connect(ui->lfArchorDown, &QRadioButton::toggled, this, &Widget::on_lfArchorDown_toggled);
    connect(ui->rfArchorDown, &QRadioButton::toggled, this, &Widget::on_rfArchorDown_toggled);
    connect(ui->lbArchorDown, &QRadioButton::toggled, this, &Widget::on_lbArchorDown_toggled);
    connect(ui->rbArchorDown, &QRadioButton::toggled, this, &Widget::on_rbArchorDown_toggled);
    connect(ui->archorResetButton, &QPushButton::clicked, this, &Widget::on_archorResetButton_clicked);
    // cable button
    connect(ui->lfPullCableButton, &QRadioButton::toggled, this, &Widget::on_lfPullCableButton_toggled);
    connect(ui->rfPullCableButton, &QRadioButton::toggled, this, &Widget::on_rfPullCableButton_toggled);
    connect(ui->lbPullCableButton, &QRadioButton::toggled, this, &Widget::on_lbPullCableButton_toggled);
    connect(ui->rbPullCableButton, &QRadioButton::toggled, this, &Widget::on_rbPullCableButton_toggled);
    connect(ui->lfReleaseCableButton, &QRadioButton::toggled, this, &Widget::on_lfReleaseCableButton_toggled);
    connect(ui->rfReleaseCableButton, &QRadioButton::toggled, this, &Widget::on_rfReleaseCableButton_toggled);
    connect(ui->lbReleaseCableButton, &QRadioButton::toggled, this, &Widget::on_lbReleaseCableButton_toggled);
    connect(ui->rbReleaseCableButton, &QRadioButton::toggled, this, &Widget::on_rbReleaseCableButton_toggled);
    connect(ui->cableResetButton, &QPushButton::clicked, this, &Widget::on_cableResetButton_clicked);
    // stop button
    connect(ui->stopButton, &QPushButton::clicked, this, &Widget::on_stopButton_clicked);
    // traj button
    connect(ui->readLocalTraj, &QPushButton::clicked, this, &Widget::on_readLocalTraj_clicked);
    connect(ui->startLocalTraj, &QPushButton::clicked, this, &Widget::on_startLocalTraj_clicked);
    // reset flag
    connect(usbcan_, &usb_can::UsbCan::resetMsg, archors_, &motor_driver::ArchorDriver::recvResetMsg);
    // connect(archors_, &motor_driver::ArchorDriver::resetFinished, local_traj_, &local_traj_ctrl::LocalTrajCtrl::);
    // connect(cables_, &motor_driver::CableDriver::resetFinished, local_traj_, &local_traj_ctrl::LocalTrajCtrl::);

    // cables_->init(motor_driver::RunMode::VEL_POS);
    // archors_->init(motor_driver::RunMode::VEL, 0x00);
}

Widget::~Widget()
{
    delete ui;
    if (archor_thread_)
    {
        archor_thread_->quit();
        archor_thread_->wait();
    }
    if (cable_thread_)
    {
        cable_thread_->quit();
        cable_thread_->wait();
    }
    if (can_thread_)
    {
        can_thread_->quit();
        can_thread_->wait();
    }
}

void Widget::on_lfPullCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = ui->lfCableVel->value();
        target_pos = ui->lfCableLen->value();
        qInfo() << "LF cable start pulling";
    }
    else
        qInfo() << "LF cable stop pulling";

    cables_->setSendVelPos(0, target_vel, target_pos);
}

void Widget::on_rfPullCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = ui->rfCableVel->value();
        target_pos = ui->rfCableLen->value();
        qInfo() << "RF cable start pulling";
    }
    else
        qInfo() << "RF cable stop pulling";

    cables_->setSendVelPos(1, target_vel, target_pos);
}

void Widget::on_lbPullCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = ui->lbCableVel->value();
        target_pos = ui->lbCableLen->value();
        qInfo() << "LB cable start pulling";
    }
    else
        qInfo() << "LB cable stop pulling";

    cables_->setSendVelPos(2, target_vel, target_pos);
}

void Widget::on_rbPullCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = -ui->rbCableVel->value();
        target_pos = ui->rbCableLen->value();
        qInfo() << "RB cable start pulling";
    }
    else
        qInfo() << "RB cable stop pulling";

    cables_->setSendVelPos(3, target_vel, target_pos);
}

void Widget::on_lfReleaseCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = -ui->lfCableVel->value();
        target_pos = ui->lfCableLen->value();
        qInfo() << "LF cable start releasing";
    }
    else
        qInfo() << "LF cable stop releasing";

    cables_->setSendVelPos(0, target_vel, target_pos);
}

void Widget::on_rfReleaseCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = -ui->rfCableVel->value();
        target_pos = ui->rfCableLen->value();
        qInfo() << "RF cable start releasing";
    }
    else
        qInfo() << "RF cable stop releasing";

    cables_->setSendVelPos(1, target_vel, target_pos);
}

void Widget::on_lbReleaseCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = -ui->lbCableVel->value();
        target_pos = ui->lbCableLen->value();
        qInfo() << "LB cable start releasing";
    }
    else
        qInfo() << "LB cable stop releasing";

    cables_->setSendVelPos(2, target_vel, target_pos);
}

void Widget::on_rbReleaseCableButton_toggled(bool checked)
{
    int target_vel = 0, target_pos = 0;

    if (true == checked)
    {
        target_vel = ui->rbCableVel->value();
        target_pos = ui->rbCableLen->value();
        qInfo() << "RB cable start releasing";
    }
    else
        qInfo() << "RB cable stop releasing";

    cables_->setSendVelPos(3, target_vel, target_pos);
}

void Widget::on_cableResetButton_clicked()
{
    std::vector<int> direction{ 1, 1, 1, -1 };

    qInfo() << "ALL cables start reseting";
    for (size_t i = 0; i < 4; ++i)
    {
        cables_->setSendVelPos(i, ui->cableResetVel->value() * direction[i], 0);
    }
}

void Widget::on_lfArchorUp_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = ui->lfArchorVel->value();
        qInfo() << "LF archor start moving up";
    }
    else
        qInfo() << "LF archor stop moving up";

    archors_->setSendVel(0, target_vel);
}

void Widget::on_rfArchorUp_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = ui->rfArchorVel->value();
        qInfo() << "RF archor start moving up";
    }
    else
        qInfo() << "RF archor stop moving up";

    archors_->setSendVel(1, target_vel);
}

void Widget::on_lbArchorUp_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = ui->lbArchorVel->value();
        qInfo() << "LB archor start moving up";
    }
    else
        qInfo() << "LB archor stop moving up";

    archors_->setSendVel(2, target_vel);
}

void Widget::on_rbArchorUp_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = ui->rbArchorVel->value();
        qInfo() << "RB archor start moving up";
    }
    else
        qInfo() << "RB archor stop moving up";

    archors_->setSendVel(3, target_vel);
}

void Widget::on_lfArchorDown_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = -ui->lfArchorVel->value();
        qInfo() << "LF archor start moving down";
    }
    else
        qInfo() << "LF archor stop moving down";

    archors_->setSendVel(0, target_vel);
}

void Widget::on_rfArchorDown_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = -ui->rfArchorVel->value();
        qInfo() << "RF archor start moving down";
    }
    else
        qInfo() << "RF archor stop moving down";

    archors_->setSendVel(1, target_vel);
}

void Widget::on_lbArchorDown_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = -ui->lbArchorVel->value();
        qInfo() << "LB archor start moving down";
    }
    else
        qInfo() << "LB archor stop moving down";

    archors_->setSendVel(2, target_vel);
}

void Widget::on_rbArchorDown_toggled(bool checked)
{
    int target_vel = 0;

    if (true == checked)
    {
        target_vel = -ui->rbArchorVel->value();
        qInfo() << "RB archor start moving down";
    }
    else
        qInfo() << "RB archor stop moving down";

    archors_->setSendVel(3, target_vel);
}

void Widget::on_archorResetButton_clicked()
{
    if (archor_thread_->isFinished())
        archor_thread_->start();
    emit startArchorReset(ui->archorResetVel->value());
}

void Widget::on_stopButton_clicked()
{
    archors_->stop();
    cables_->stop();
}

void Widget::on_startLocalTraj_clicked()
{
    if (archor_thread_->isFinished())
        archor_thread_->start();
    if (cable_thread_->isFinished())
        cable_thread_->start();
    emit startArchorRunTraj(ui->archorResetVel->value(), ui->localTrajPeriod->value());
    emit startCableRunTraj(ui->localTrajPeriod->value());
}

void Widget::on_readLocalTraj_clicked()
{
    QString traj = ui->localTraj->currentText();
    QString path{};
    if ("上下轨迹" == traj)
        path = QCoreApplication::applicationDirPath() + "/../csv/updown.csv";
    else if ("斜线轨迹" == traj)
        path = QCoreApplication::applicationDirPath() + "/../csv/line.csv";
    else if ("圆轨迹" == traj)
        path = QCoreApplication::applicationDirPath() + "/../csv/circle.csv";

    local_traj_.readTraj(path, archor_traj_, cable_traj_);

    qDebug() << archor_traj_;
}
