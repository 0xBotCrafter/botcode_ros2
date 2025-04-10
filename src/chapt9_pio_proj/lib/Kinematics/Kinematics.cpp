#include "Kinematics.hpp"

void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance)
{
    motor_param_[id].per_pulse_distance = per_pulse_distance;
}

void Kinematics::set_wheel_distance(float wheel_distance)
{
    wheel_distance_ = wheel_distance;
}

int16_t Kinematics::get_motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed;
}

void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick)
{
    // 计算 dt
    uint32_t dt = current_time - last_update_time;
    last_update_time = current_time;

    // 计算编码器差值dtick
    int32_t dtick_left = left_tick - motor_param_[0].last_encoder_tick;
    int32_t dtick_right = right_tick - motor_param_[1].last_encoder_tick;
    motor_param_[0].last_encoder_tick = left_tick;
    motor_param_[1].last_encoder_tick = right_tick;

    // 计算速度 mm/s
    motor_param_[0].motor_speed = float(dtick_left * motor_param_[0].per_pulse_distance) / dt * 1000;
    motor_param_[1].motor_speed = float(dtick_right * motor_param_[1].per_pulse_distance) / dt * 1000;

    // 更新里程计信息
    update_odom(dt);
}

void Kinematics::kinematics_forward(float left_speed, float right_speed, float &out_linear_speed, float &out_angle_speed)
{
    out_linear_speed = (right_speed + left_speed) / 2.0;
    out_angle_speed = (right_speed - left_speed) / wheel_distance_;
}

void Kinematics::kinematics_inverse(float linear_speed, float angle_speed, float &out_left_speed, float &out_right_speed)
{
    out_left_speed = linear_speed - angle_speed * wheel_distance_ / 2.0;
    out_right_speed = linear_speed + angle_speed * wheel_distance_ / 2.0;
}

void Kinematics::update_odom(uint16_t dt)
{
    // ms to s
    float dt_s = float(dt) / 1000.0;
    // 运动学正解
    this->kinematics_forward(motor_param_[0].motor_speed, motor_param_[1].motor_speed, odom_.linear_speed, odom_.angle_speed);
    // 转换线速度单位 mm/s -> m/s
    odom_.linear_speed /= 1000.0;
    // 计算当前角度
    odom_.angle += odom_.angle_speed * dt_s;
    // 角度odom_.yaw限制在-pi到pi之间
    Kinematics::TransAngleInPI(odom_.angle, odom_.angle);

    // 计算机器人移动距离和在两轴上的分量并进行累积
    float delta_distance = odom_.linear_speed * dt_s;
    odom_.x += delta_distance * cos(odom_.angle);
    odom_.y += delta_distance * sin(odom_.angle);
}

odom_t &Kinematics::get_odom()
{
    return odom_;
}

void Kinematics::TransAngleInPI(float &angle, float &out_angle)
{
    while (angle > PI)
    {
        out_angle -= 2 * PI;
    }
    while (angle < -PI)
    {
        out_angle += 2 * PI;
    }
}
