#include <utilities/angles.hpp>

#include "chassis_ctrl.hpp"

using namespace chassis_ctrl;

ChassisCtrl::ChassisCtrl(QObject* parent)
{
    Q_ASSERT(4 == params_.position.size() && 4 == params_.roll_dir.size() && 4 == params_.steer_dir.size());
    for (size_t i = 0; i < 4; ++i)
    {
        wheelsets_.push_back(Wheelset{ .position_ = Vec2<double>{ params_.position[i][0], params_.position[i][1] },
                                       .roll_direction_ = params_.roll_dir[i],
                                       .steer_direction_ = params_.steer_dir[i],
                                       .wheel_radius_ = params_.radius_,
                                       .last_angle_ = 0,
                                       .steer_state_ = 0,
                                       .ramp_angle_ = std::make_shared<RampFilter<double>>(M_PI, 0.05),
                                       .ramp_vel_ = std::make_shared<RampFilter<double>>(params_.accel_, 0.005) });
    }

    // initialize member variables
    steer_cmd_.resize(wheelsets_.size());
    roll_cmd_.resize(wheelsets_.size());
}

void ChassisCtrl::updateVel(const double& x, const double& y, const double& turn)
{
    cmd_vel_.x = x;
    cmd_vel_.y = y;
    cmd_vel_.turn = turn;
}

void ChassisCtrl::updateSteerState(const std::vector<double>& state)
{
    for (size_t i = 0; i < state.size(); ++i)
        wheelsets_[i].steer_state_ = state[i];
}

void ChassisCtrl::stopRun()
{
    stop_run_ = false;
}

// Ref: https://dominik.win/blog/programming-swerve-drive/
// Ref: https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
void ChassisCtrl::move()
{
    double target_angle = 0.0, target_vel = 0.0;
    Vec2<double> vel_center;

    while (!stop_run_)
    {
        vel_center << cmd_vel_.x, cmd_vel_.y;  // velocity of wheelset
        for (auto& wheelset : wheelsets_)
        {
            if (0 == cmd_vel_.x && 0 == cmd_vel_.y && 0 == cmd_vel_.turn)
            {
                // If all velocity commands are 0, the wheelset maintains the current angle and the velocity reduced
                target_angle = wheelset.last_angle_;
                target_vel = 0;
            }
            else
            {
                // Calculate the speed and angle of each wheelset based on the speed of car center
                Vec2<double> vel =
                    vel_center + cmd_vel_.turn * Vec2<double>(-wheelset.position_.y(), wheelset.position_.x());
                double vel_angle = std::atan2(vel.y(), vel.x());

                // Direction flipping and Stray wheelset mitigation
                double a = angles::shortest_angular_distance(wheelset.steer_state_, vel_angle);
                double b = angles::shortest_angular_distance(wheelset.steer_state_, vel_angle + M_PI);
                if (std::abs(a) > std::abs(b))
                    target_angle =
                        (vel_angle + M_PI) > M_PI ? angles::normalize_angle(vel_angle + M_PI) : vel_angle + M_PI;
                else
                    target_angle = vel_angle;
                wheelset.last_angle_ = target_angle;
                target_vel = vel.norm() / wheelset.wheel_radius_ * std::cos(a) * wheelset.roll_direction_;
            }

            // Smoothing steering angle and rolling velocity
            wheelset.ramp_angle_->input(target_angle);
            wheelset.ramp_vel_->input(target_vel);
            steer_cmd_.push_back(wheelset.ramp_angle_->output());
            roll_cmd_.push_back(wheelset.ramp_vel_->output());
        }

        emit sendA1Vel(roll_cmd_);
        emit sendGOPos(steer_cmd_);
        steer_cmd_.clear();
        roll_cmd_.clear();
    }
}
