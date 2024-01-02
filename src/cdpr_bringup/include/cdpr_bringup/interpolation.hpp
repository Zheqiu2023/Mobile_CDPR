/**
 * @File Name: interpolation.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-06-20
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#pragma once

#include <assert.h>
#include <type_traits>
#include <vector>
#include <math.h>

namespace interpolate
{
/**
 * @brief Linear interpolation between y0 and yf.  x is between 0 and 1
 * @tparam y_t
 * @tparam x_t
 * @param  y0
 * @param  yf
 * @param  x
 * @return y_t
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x)
{
    static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
    assert(x >= 0 && x <= 1);
    return y0 + (yf - y0) * x;
}

// 以下为三阶贝塞尔曲线插值算法
// 令三阶贝塞尔的中间控制点P1,P2分别与P0,P3重合,造成的效果为:起点终点速度为0，若想让终点保持一定速度,改变P3
// 起点处速度为3(P1-P0)/swing_time,终点处速度为:3(P3-P2)/swing_time

/**
 * @brief Cubic bezier interpolation between y0 and yf.  x is between 0 and 1，获取位置
 * @tparam y_t
 * @tparam x_t
 * @param  y0
 * @param  yf
 * @param  x
 * @return y_t
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x)
{
    static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
    assert(x >= 0 && x <= 1);
    y_t yDiff = yf - y0;
    x_t bezier = static_cast<x_t>(3) * x * x - static_cast<x_t>(2) * x * x * x;
    return y0 + bezier * yDiff;  // 三阶贝塞尔曲线公式（令P0=P1，P2=P3）
}

/**
 * @brief 贝塞尔公式一阶微分用于获取速度
 * @tparam y_t
 * @tparam x_t
 * @param  y0
 * @param  yf
 * @param  x
 * @return y_t
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x)
{
    static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
    assert(x >= 0 && x <= 1);
    y_t yDiff = yf - y0;
    x_t bezier = static_cast<x_t>(6) * x * (static_cast<x_t>(1) - x);
    return bezier * yDiff;
}

/**
 * @brief 贝塞尔公式二阶微分用于获取加速度
 * @tparam y_t
 * @tparam x_t
 * @param  y0
 * @param  yf
 * @param  x
 * @return y_t
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x)
{
    static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
    assert(x >= 0 && x <= 1);
    y_t yDiff = yf - y0;
    x_t bezier = static_cast<x_t>(6) - static_cast<x_t>(12) * x;
    return bezier * yDiff;
}

/**
 * @brief 贝塞尔轨迹规划器
 * @param  start_pos
 * @param  end_pos
 * @param  phase
 * @param  total_run_time
 * @return std::vector<double>
 */
std::vector<double> cubicBezierTrajPlanner(const double& start_pos, const double& end_pos, const double& phase,
                                           const double& total_run_time)
{
    std::vector<double> result_vec(3, 0);
    result_vec[0] = cubicBezier(start_pos, end_pos, phase);  // Pos
    result_vec[1] =
        cubicBezierFirstDerivative(start_pos, end_pos, phase) /
        total_run_time;  // Vec.因为phase的值位0-1,相当于单位时间,因此需要除设定的轨迹运行时间以将其转换成在轨迹运行时间上的速度
    result_vec[2] = cubicBezierSecondDerivative(start_pos, end_pos, phase) / (total_run_time * total_run_time);  // Acc
    return result_vec;
}

/**
 * @brief 五次多项式插值算法,需提供起点和终点信息
 * @param  time
 * @param  pos
 * @param  vel
 * @param  acc
 * @param  point_num
 * @return std::vector<std::vector<double>>
 */
std::vector<std::vector<double>> quinticPolynomial(const double& time, const std::vector<double>& pos,
                                                   const std::vector<double>& vel, const std::vector<double>& acc,
                                                   const int& point_num)
{
    std::vector<double> coef_vec(6, 0);
    std::vector<std::vector<double>> result_vec(point_num, std::vector<double>(3, 0));
    double pos_diff = pos[1] - pos[0];

    // 五次多项式系数
    coef_vec[0] = pos[0];
    coef_vec[1] = vel[0];
    coef_vec[2] = 1.0 / 2 * acc[0];
    coef_vec[3] = 1.0 / (2 * pow(time, 3)) *
                  (20 * pos_diff - (8 * vel[1] + 12 * vel[0]) * time + (acc[1] - 3 * acc[0]) / pow(time, 2));
    coef_vec[4] = 1.0 / (2 * pow(time, 4)) *
                  (-30 * pos_diff + (14 * vel[1] + 16 * vel[0]) * time + (3 * acc[0] - 2 * acc[1]) / pow(time, 2));
    coef_vec[5] =
        1.0 / (2 * pow(time, 5)) * (12 * pos_diff - 6 * (vel[1] + vel[0]) * time + (acc[1] - acc[0]) / pow(time, 2));

    double per_seg_time = time / (point_num - 1);
    for (size_t i = 0; i < point_num; ++i)
    {
        // 位置
        result_vec[i][0] = coef_vec[0] + coef_vec[1] * pow(per_seg_time * i, 1) +
                           coef_vec[2] * pow(per_seg_time * i, 2) + coef_vec[3] * pow(per_seg_time * i, 3) +
                           coef_vec[4] * pow(per_seg_time * i, 4) + coef_vec[5] * pow(per_seg_time * i, 5);
        // 速度
        result_vec[i][1] = coef_vec[1] + 2 * coef_vec[2] * pow(per_seg_time * i, 1) +
                           3 * coef_vec[3] * pow(per_seg_time * i, 2) + 4 * coef_vec[4] * pow(per_seg_time * i, 3) +
                           5 * coef_vec[5] * pow(per_seg_time * i, 4);
        // 加速度
        result_vec[i][2] = 2 * coef_vec[2] + 6 * coef_vec[3] * pow(per_seg_time * i, 1) +
                           12 * coef_vec[4] * pow(per_seg_time * i, 2) + 20 * coef_vec[5] * pow(per_seg_time * i, 3);
    }
    return result_vec;
}
}  // namespace interpolate