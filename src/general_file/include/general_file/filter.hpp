/**
 * @File Name: filter.hpp
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
#include <vector>
#include <numeric>
#include <algorithm>

namespace filter
{
/**
 * @brief 中位均值滤波：删除最大最小元素，再求和取平均值
 * @tparam T
 * @param  vec
 * @return T
 */
template <typename T>
T medianMeanFilter(std::vector<T>& vec)
{
    sort(vec.begin(), vec.end());
    vec.erase(vec.begin());
    vec.pop_back();
    T sum = accumulate(vec.begin(), vec.end(), T(0));
    return sum / vec.size();
};
}  // namespace filter