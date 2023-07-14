/**
 * @File Name: singleton_template.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-26
 *
 *  ***********************************************************************************
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 *  ***********************************************************************************
 */
#pragma once

// 单例模式：确保类的实例化对象只有一个
template <typename T>
class Singleton
{
  public:
    Singleton() = default;
    virtual ~Singleton() = default;
    Singleton(const Singleton&) = delete;
    Singleton(const Singleton&&) = delete;
    Singleton operator=(const Singleton&) = delete;

    static T& getInstance()
    {
        static T instance;
        return instance;
    }
};