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
    can_thread_ = new QThread;
    archor_thread_ = new QThread;
    cable_thread_ = new QThread;
    a1_thread_ = new QThread;
    go_thread_ = new QThread;
    chassis_thread_ = new QThread;
    // 创建任务对象
    usbcan_ = new usb_can::UsbCan;
    archors_ = new motor_driver::ArchorDriver;
    cables_ = new motor_driver::CableDriver;
    a1_ = new motor_a1::A1Driver;
    go_ = new motor_go::GoDriver;
    chassis_ = new chassis_ctrl::ChassisCtrl;
    // 移动任务对象到子线程中
    archors_->moveToThread(archor_thread_);
    cables_->moveToThread(cable_thread_);
    usbcan_->moveToThread(can_thread_);
    a1_->moveToThread(a1_thread_);
    go_->moveToThread(go_thread_);
    chassis_->moveToThread(chassis_thread_);
    // 释放线程资源
    connect(archor_thread_, &QThread::finished, archors_, &QObject::deleteLater);  // 释放任务对象
    connect(archor_thread_, &QThread::finished, archor_thread_,
            &QObject::deleteLater);  // 释放子线程, 也可 archor_thread_ = new QThread(this)
    connect(cable_thread_, &QThread::finished, cables_, &QObject::deleteLater);
    connect(cable_thread_, &QThread::finished, cable_thread_, &QObject::deleteLater);
    connect(can_thread_, &QThread::finished, usbcan_, &QObject::deleteLater);
    connect(can_thread_, &QThread::finished, can_thread_, &QObject::deleteLater);
    connect(a1_thread_, &QThread::finished, a1_, &QObject::deleteLater);
    connect(a1_thread_, &QThread::finished, a1_thread_, &QObject::deleteLater);
    connect(go_thread_, &QThread::finished, go_, &QObject::deleteLater);
    connect(go_thread_, &QThread::finished, go_thread_, &QObject::deleteLater);
    connect(chassis_thread_, &QThread::finished, chassis_, &QObject::deleteLater);
    connect(chassis_thread_, &QThread::finished, chassis_thread_, &QObject::deleteLater);
    // 连接信号与槽
    connect(this, &Widget::startArchorReset, archors_, &motor_driver::ArchorDriver::reset);
    connect(this, &Widget::startArchorRunTraj, archors_, &motor_driver::ArchorDriver::runTraj);
    connect(this, &Widget::startCableRunTraj, cables_, &motor_driver::CableDriver::runTraj);
    connect(this, &Widget::startCanRecvPos, usbcan_, &usb_can::UsbCan::recvPos);
    connect(this, &Widget::startA1RunTraj, a1_, &motor_a1::A1Driver::runTraj);
    connect(this, &Widget::startGORunTraj, go_, &motor_go::GoDriver::runTraj);

    connect(this, &Widget::startRemoteControl, chassis_, &chassis_ctrl::ChassisCtrl::move);

    connect(this, &Widget::sendChassisVel, chassis_, &chassis_ctrl::ChassisCtrl::updateVel);
    connect(chassis_, &chassis_ctrl::ChassisCtrl::sendA1Vel, a1_, &motor_a1::A1Driver::recvVel);
    connect(chassis_, &chassis_ctrl::ChassisCtrl::sendGOVel, go_, &motor_go::GoDriver::recvAngle);
    connect(go_, &motor_go::GoDriver::sendSteerState, chassis_, &chassis_ctrl::ChassisCtrl::updateSteerState);

    // mode button
    connect(ui->cableVelMode, &QRadioButton::toggled, this, &Widget::selectCableMode);
    connect(ui->cableVelPosMode, &QRadioButton::toggled, this, &Widget::selectCableMode);
    connect(ui->cableCurMode, &QRadioButton::toggled, this, &Widget::selectCableMode);

    connect(ui->archorVelMode, &QRadioButton::toggled, this, &Widget::selectArchorMode);
    connect(ui->archorVelPosMode, &QRadioButton::toggled, this, &Widget::selectArchorMode);

    connect(ui->remoteControl, &QRadioButton::toggled, this, &Widget::selectChassisMode);
    connect(ui->trajTracking, &QRadioButton::toggled, this, &Widget::selectChassisMode);

    // 初始化QButtonGroup，添加相应的QRadioButton并设置ID
    cableModeGroup = new QButtonGroup(this);
    archorModeGroup = new QButtonGroup(this);
    chassisModeGroup = new QButtonGroup(this);

    cableModeGroup->addButton(ui->cableVelMode, 0);
    cableModeGroup->addButton(ui->cableVelPosMode, 1);
    cableModeGroup->addButton(ui->cableCurMode, 2);
    ui->cableVelPosMode->setChecked(true);

    archorModeGroup->addButton(ui->archorVelMode, 0);
    archorModeGroup->addButton(ui->archorVelPosMode, 1);
    ui->archorVelMode->setChecked(true);

    chassisModeGroup->addButton(ui->remoteControl, 0);
    chassisModeGroup->addButton(ui->trajTracking, 1);
}

Widget::~Widget()
{
    delete ui;

    archor_thread_->quit();
    archor_thread_->wait();

    cable_thread_->quit();
    cable_thread_->wait();

    can_thread_->quit();
    can_thread_->wait();

    a1_thread_->quit();
    a1_thread_->wait();

    go_thread_->quit();
    go_thread_->wait();

    chassis_thread_->quit();
    chassis_thread_->wait();
}

/****Mode
 * Button**************************************************************************************************************************************************/

void Widget::selectCableMode(bool checked)
{
    if (checked)
    {
        if (0 == cableModeGroup->checkedId())
            cables_->init(motor_driver::RunMode::VEL);
        else if (1 == cableModeGroup->checkedId())
            cables_->init(motor_driver::RunMode::VEL_POS);
        else if (2 == cableModeGroup->checkedId())
            cables_->init(motor_driver::RunMode::CUR);
    }
}

void Widget::selectArchorMode(bool checked)
{
    if (checked)
    {
        if (0 == archorModeGroup->checkedId())
            archors_->init(motor_driver::RunMode::VEL);
        else if (1 == archorModeGroup->checkedId())
            archors_->init(motor_driver::RunMode::VEL_POS);
    }
}

void Widget::selectChassisMode(bool checked)
{
    if (checked)
    {
        if (0 == chassisModeGroup->checkedId())
        {
            this->grabKeyboard();
            chassis_thread_->start();

            qInfo() << Ui::REMOTE_CONTROL_MSG;
        }
        else if (1 == chassisModeGroup->checkedId())
        {
            chassis_->stopRun();
            a1_->setCmd(std::vector<double>{ ui->a1VelKw->value() });
            go_->setCmd(std::vector<double>{ ui->goPosKp->value(), ui->goPosKw->value() });
        }
    }
}

/****Traj
 * Button********************************************************************************************************************************************/

void Widget::on_readLocalTraj_clicked()
{
    QString path{};
    if (0 == ui->localTraj->currentIndex())  // updown traj
        path = QCoreApplication::applicationDirPath() + "/../csv/local/updown.csv";
    else if (1 == ui->localTraj->currentIndex())  // line traj
        path = QCoreApplication::applicationDirPath() + "/../csv/local/line.csv";
    else if (2 == ui->localTraj->currentIndex())  // circle traj
        path = QCoreApplication::applicationDirPath() + "/../csv/local/circle.csv";

    archor_traj_.clear();
    cable_traj_.clear();
    local_traj_.readTraj(path, archor_traj_, cable_traj_);
}

void Widget::on_startLocalTraj_clicked()
{
    if (!(ui->cableVelPosMode->isChecked() && ui->archorVelPosMode->isChecked()))
    {
        QMessageBox::critical(this, "Run Mode", "Archors/cables not in right mode");
        return;
    }
    can_thread_->start();
    archor_thread_->start();
    cable_thread_->start();
    emit startCanRecvPos();
    emit startArchorRunTraj(ui->localTrajPeriod->value(), archor_traj_);
    emit startCableRunTraj(ui->localTrajPeriod->value(), cable_traj_);
}

void Widget::on_stopLocalTraj_clicked()
{
    usbcan_->stopRun();
    archors_->stopRun();
    cables_->stopRun();

    qInfo("stop running local traj");
}

void Widget::on_readGlobalTraj_clicked()
{
    QString traj = ui->localTraj->currentText();
    QString path1{}, path2{};
    if ("斜线轨迹" == traj)
    {
        path1 = QCoreApplication::applicationDirPath() + "/../csv/global/cdpr1.csv";  // cabel&archor
        path2 = QCoreApplication::applicationDirPath() + "/../csv/global/cdpr2.csv";  // chassis
    }

    global_traj_.readTraj(path1, path2, archor_traj_, cable_traj_, a1_traj_, go_traj_);
}

void Widget::on_startGlobalTraj_clicked()
{
    if (!(ui->cableVelPosMode->isChecked() && ui->archorVelPosMode->isChecked() && ui->trajTracking->isChecked()))
    {
        QMessageBox::critical(this, "Run Mode", "Archors/cables/chassis not in right mode");
        return;
    }
    can_thread_->start();
    a1_thread_->start();
    go_thread_->start();
    archor_thread_->start();
    cable_thread_->start();
    emit startCanRecvPos();
    emit startA1RunTraj(ui->globalTrajPeriod->value(), a1_traj_);
    emit startGORunTraj(ui->globalTrajPeriod->value(), go_traj_);
    emit startArchorRunTraj(ui->globalTrajPeriod->value(), archor_traj_);
    emit startCableRunTraj(ui->globalTrajPeriod->value(), cable_traj_);
}

void Widget::on_stopGlobalTraj_clicked()
{
    usbcan_->stopRun();
    archors_->stopRun();
    cables_->stopRun();
    a1_->stopRun();
    go_->stopRun();
}
