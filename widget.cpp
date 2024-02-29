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
    // 创建任务对象
    usbcan_ = new usb_can::UsbCan;
    archors_ = new motor_driver::ArchorDriver;
    cables_ = new motor_driver::CableDriver;

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
    connect(this, &Widget::startCanRecvPos, usbcan_, &usb_can::UsbCan::recvPos);

    // mode button
    connect(ui->cableVelMode, &QRadioButton::toggled, this, &Widget::selectCableMode);
    connect(ui->cableVelPosMode, &QRadioButton::toggled, this, &Widget::selectCableMode);
    connect(ui->archorVelMode, &QRadioButton::toggled, this, &Widget::selectArchorMode);
    connect(ui->archorVelPosMode, &QRadioButton::toggled, this, &Widget::selectArchorMode);

    // 初始化QButtonGroup，添加相应的QRadioButton并设置ID
    cableModeGroup = new QButtonGroup(this);
    archorModeGroup = new QButtonGroup(this);
    cableModeGroup->addButton(ui->cableVelMode, 0);
    cableModeGroup->addButton(ui->cableVelPosMode, 1);
    ui->cableVelPosMode->setChecked(true);
    archorModeGroup->addButton(ui->archorVelMode, 0);
    archorModeGroup->addButton(ui->archorVelPosMode, 1);
    ui->archorVelMode->setChecked(true);
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
}

void Widget::on_lfCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for cables");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (true == checked)
    {
        target_vel = ui->lfCableVel->value();
        target_pos = cables_->convertPos(ui->lfCableLen->value() / double(1000));
        qInfo() << "LF cable start pulling";
    }
    else
        qInfo() << "LF cable stop pulling";

    cables_->setSendVelPos(0, target_vel, target_pos);
}

void Widget::on_rfCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for cables");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (true == checked)
    {
        target_vel = ui->rfCableVel->value();
        target_pos = cables_->convertPos(-ui->rfCableLen->value() / double(1000));
        qInfo() << "RF cable start pulling";
    }
    else
        qInfo() << "RF cable stop pulling";

    cables_->setSendVelPos(1, target_vel, target_pos);
}

void Widget::on_lbCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for cables");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (true == checked)
    {
        target_vel = ui->lbCableVel->value();
        target_pos = cables_->convertPos(-ui->lbCableLen->value() / double(1000));
        qInfo() << "LB cable start pulling";
    }
    else
        qInfo() << "LB cable stop pulling";

    cables_->setSendVelPos(2, target_vel, target_pos);
}

void Widget::on_rbCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for cables");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (true == checked)
    {
        target_vel = -ui->rbCableVel->value();
        target_pos = cables_->convertPos(ui->rbCableLen->value() / double(1000));
        qInfo() << "RB cable start pulling";
    }
    else
        qInfo() << "RB cable stop pulling";

    cables_->setSendVelPos(3, target_vel, target_pos);
}

void Widget::on_cableResetButton_clicked()
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for cables");
        return;
    }

    std::vector<int> direction{ 1, 1, 1, -1 };
    qInfo() << "ALL cables start reseting";
    for (size_t i = 0; i < 4; ++i)
    {
        cables_->setSendVelPos(i, ui->cableResetVel->value() * direction[i], 0);
    }
}

void Widget::on_lfArchorUp_toggled(bool checked)
{
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

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
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors");
        return;
    }

    can_thread_->start();
    archor_thread_->start();
    emit startArchorReset(ui->archorResetVel->value());
}

void Widget::on_stopButton_clicked()
{
    usbcan_->stop();
    archors_->stop();
    cables_->stop();
}

void Widget::on_startLocalTraj_clicked()
{
    if (!(ui->cableVelPosMode->isChecked() && ui->archorVelPosMode->isChecked()))
    {
        QMessageBox::critical(this, "run mode", "Wrong motor mode for archors or cables");
        return;
    }
    can_thread_->start();
    archor_thread_->start();
    cable_thread_->start();
    emit startCanRecvPos();
    emit startArchorRunTraj(ui->localTrajPeriod->value(), archor_traj_);
    emit startCableRunTraj(ui->localTrajPeriod->value(), cable_traj_);
}

void Widget::on_readLocalTraj_clicked()
{
    QString traj = ui->localTraj->currentText();
    QString path{};
    if ("竖直轨迹" == traj)
        path = QCoreApplication::applicationDirPath() + "/../csv/updown.csv";
    else if ("斜线轨迹" == traj)
        path = QCoreApplication::applicationDirPath() + "/../csv/line.csv";
    else if ("圆轨迹" == traj)
        path = QCoreApplication::applicationDirPath() + "/../csv/circle.csv";

    local_traj_.readTraj(path, archor_traj_, cable_traj_);
}

void Widget::selectCableMode(bool checked)
{
    if (checked)
    {
        if (0 == cableModeGroup->checkedId())
            cables_->init(motor_driver::RunMode::VEL);
        else if (1 == cableModeGroup->checkedId())
            cables_->init(motor_driver::RunMode::VEL_POS);
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
