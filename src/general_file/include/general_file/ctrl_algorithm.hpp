/**
 * @File Name: simple_algorithm.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-06-28
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#pragma once

namespace ctrl_algorithm
{
class PID
{
  public:
    void pidConfig(const float& kp, const float& ki, const float& kd, const float& integral_lim, const int& frequency)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        integral_lim_ = integral_lim;
        freq_ = frequency;
    }

    float pidProcess(const float& target_val, const float& actual_val)
    {
        float error = target_val - actual_val;

        if (error < 5 && error > -5)
        {
            integral_ += error;
            integral_ = integral_ > integral_lim_ ? integral_lim_ : integral_;
            integral_ = integral_ < -integral_lim_ ? -integral_lim_ : integral_;
        }
        else
            integral_ = 0;

        float derivative = (error - last_error_) * freq_;
        float output = kp_ * error + ki_ * integral_ / freq_ + kd_ * derivative;

        last_error_ = error;

        return output;
    }

  private:
    float kp_ = 0, ki_ = 0, kd_ = 0;
    float last_error_ = 0;
    float integral_ = 0, integral_lim_ = 0;
    int freq_ = 0;
};
}  // namespace ctrl_algorithm