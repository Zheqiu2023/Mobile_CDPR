#include "widget.hpp"
#include "ui_widget.h"

#include <QDebug>
#include <QSpinBox>
#include <QRadioButton>
#include <QPushButton>
#include <QMessageBox>

/****Reset
 * Button******************************************************************************************************************************************/

void Widget::on_cableZeroPoint_clicked()
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_position mode");
        return;
    }

    qInfo() << "ALL cables start moving back to zero point";
    for (size_t i = 0; i < 4; ++i)
        cables_->setSendVelPos(i, ui->cableResetVel->value(), 0);
}

void Widget::on_archorBottom_clicked()
{
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    can_thread_->start();
    archor_thread_->start();
    emit startArchorReset(ui->archorResetVel->value());
}

void Widget::on_archorZeroPoint_clicked()
{
    if (!ui->archorVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity_position mode");
        return;
    }

    qInfo() << "ALL archors start moving back to zero point";
    for (size_t i = 0; i < 4; ++i)
        archors_->setSendVelPos(i, ui->archorResetVel->value(), 0);
}

void Widget::on_archorStopResetButton_clicked()
{
    archors_->stopReset();
}

/****Jog
 * Button******************************************************************************************************************************************/

void Widget::on_lfCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_position mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->lfCableVel->value();
        target_pos = cables_->convertPos(ui->lfCableLen->value() / double(1000));
        if (target_pos >= 0)
            qInfo() << "LF cable start pulling";
        else
            qInfo() << "LF cable start releasing";
    }
    else
        qInfo() << "LF cable stop moving";

    cables_->setSendVelPos(0, target_vel, target_pos);
}

void Widget::on_rfCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_position mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->rfCableVel->value();
        target_pos = cables_->convertPos(-ui->rfCableLen->value() / double(1000));
        if (target_pos >= 0)
            qInfo() << "RF cable start pulling";
        else
            qInfo() << "RF cable start releasing";
    }
    else
        qInfo() << "RF cable stop moving";

    cables_->setSendVelPos(1, target_vel, target_pos);
}

void Widget::on_lbCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_position mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->lbCableVel->value();
        target_pos = cables_->convertPos(-ui->lbCableLen->value() / double(1000));
        if (target_pos >= 0)
            qInfo() << "LB cable start pulling";
        else
            qInfo() << "LB cable start releasing";
    }
    else
        qInfo() << "LB cable stop moving";

    cables_->setSendVelPos(2, target_vel, target_pos);
}

void Widget::on_rbCableJogButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_position mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->rbCableVel->value();
        target_pos = cables_->convertPos(ui->rbCableLen->value() / double(1000));
        if (target_pos >= 0)
            qInfo() << "RB cable start pulling";
        else
            qInfo() << "RB cable start releasing";
    }
    else
        qInfo() << "RB cable stop moving";

    cables_->setSendVelPos(3, target_vel, target_pos);
}

/****Archor Updown
 * Button******************************************************************************************************************************************/

void Widget::on_lfArchorUp_toggled(bool checked)
{
    if (!ui->archorVelMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
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
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
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
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
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
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
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
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
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
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
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
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
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
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity mode");
        return;
    }

    int target_vel = 0;
    if (checked)
    {
        target_vel = -ui->rbArchorVel->value();
        qInfo() << "RB archor start moving down";
    }
    else
        qInfo() << "RB archor stop moving down";

    archors_->setSendVel(3, target_vel);
}

/****Locate
 * Button******************************************************************************************************************************************/

void Widget::on_lfArchorLocateButton_toggled(bool checked)
{
    if (!ui->archorVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->lfArchorLocateVel->value();
        target_pos = archors_->convertPos(ui->lfArchorInitHeight->value());
        qInfo() << "LF archor start locating";
    }
    else
        qInfo() << "LF archor stop locating";

    archors_->setSendVelPos(0, target_vel, target_pos);
}

void Widget::on_rfArchorLocateButton_toggled(bool checked)
{
    if (!ui->archorVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->rfArchorLocateVel->value();
        target_pos = archors_->convertPos(ui->rfArchorInitHeight->value());
        qInfo() << "RF archor start locating";
    }
    else
        qInfo() << "RF archor stop locating";

    archors_->setSendVelPos(1, target_vel, target_pos);
}

void Widget::on_lbArchorLocateButton_toggled(bool checked)
{
    if (!ui->archorVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->lbArchorLocateVel->value();
        target_pos = archors_->convertPos(ui->lbArchorInitHeight->value());
        qInfo() << "LB archor start locating";
    }
    else
        qInfo() << "LB archor stop locating";

    archors_->setSendVelPos(2, target_vel, target_pos);
}

void Widget::on_rbArchorLocateButton_toggled(bool checked)
{
    if (!ui->archorVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Archors not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->rbArchorLocateVel->value();
        target_pos = archors_->convertPos(ui->rbArchorInitHeight->value());
        qInfo() << "RB archor start locating";
    }
    else
        qInfo() << "RB archor stop locating";

    archors_->setSendVelPos(3, target_vel, target_pos);
}

void Widget::on_lfCableLocateButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->lfCableLocateVel->value();
        target_pos = cables_->convertPos(-ui->lfCableInitLen->value() + ui->lfCableInitLen->maximum());
        qInfo() << "LF cable start locating" << target_pos;
    }
    else
        qInfo() << "LF cable stop locating";

    cables_->setSendVelPos(0, target_vel, target_pos);
}

void Widget::on_rfCableLocateButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->rfCableLocateVel->value();
        target_pos = cables_->convertPos(ui->rfCableInitLen->value() - ui->rfCableInitLen->maximum());
        qInfo() << "RF cable start locating";
    }
    else
        qInfo() << "RF cable stop locating";

    cables_->setSendVelPos(1, target_vel, target_pos);
}

void Widget::on_lbCableLocateButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->lbCableLocateVel->value();
        target_pos = cables_->convertPos(ui->lbCableInitLen->value() - ui->lbCableInitLen->maximum());
        qInfo() << "LB cable start locating";
    }
    else
        qInfo() << "LB cable stop locating";

    cables_->setSendVelPos(2, target_vel, target_pos);
}

void Widget::on_rbCableLocateButton_toggled(bool checked)
{
    if (!ui->cableVelPosMode->isChecked())
    {
        QMessageBox::critical(this, "Run Mode", "Cables not in velocity_positoin mode");
        return;
    }

    int target_vel = 0, target_pos = 0;
    if (checked)
    {
        target_vel = ui->rbCableLocateVel->value();
        target_pos = cables_->convertPos(-ui->rbCableInitLen->value() + ui->rbCableInitLen->maximum());
        qInfo() << "RB cable start locating";
    }
    else
        qInfo() << "RB cable stop locating";

    cables_->setSendVelPos(3, target_vel, target_pos);
}
