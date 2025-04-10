#include "Arduino.h"
#include "PidController.hpp"

PidController::PidController(float kp, float ki, float kd)
{
    reset();              // 重置PID
    set_kpid(kp, ki, kd); // 更新PID参数
}

float PidController::update(float current)
{
    // 计算误差及其变化率
    float error = target_ - current;   // 计算误差
    derror_ = error - error_last_;     // 计算误差变化率
    sum_error_ += error;               // 计算积分项
    if (sum_error_ > sum_error_limit_) // 积分限幅
        sum_error_ = sum_error_limit_;
    if (sum_error_ < -sum_error_limit_) // 积分限幅
        sum_error_ = -sum_error_limit_;

    error_last_ = error; // 更新

    // 计算控制输出值
    float output = kp_ * error + ki_ * sum_error_ + kd_ * derror_;

    // 限幅输出值
    if (output > out_max_) // 限幅
        output = out_max_;
    if (output < out_min_) // 限幅
        output = out_min_;

    return output;
}

void PidController::set_target(float target)
{
    target_ = target; // 更新目标值
}

void PidController::set_kpid(float kp, float ki, float kd)
{
    kp_ = kp; // 更新PID参数
    ki_ = ki; // 更新PID参数
    kd_ = kd; // 更新PID参数
}

void PidController::reset()
{
    target_ = 0.0f;     // 控制目标值
    out_min_ = 0.0f;    // 输出最小值
    out_max_ = 0.0f;    // 输出最大值
    kp_ = 0.0f;         // 比例系数
    ki_ = 0.0f;         // 积分系数
    kd_ = 0.0f;         // 微分系数
    sum_error_ = 0.0f;  // 误差累积
    derror_ = 0.0f;     // 误差变化率
    error_last_ = 0.0f; // 上次误差
}

void PidController::set_limit_out(float out_min, float out_max)
{
    out_min_ = out_min; // 设置限幅
    out_max_ = out_max; // 设置限幅
}