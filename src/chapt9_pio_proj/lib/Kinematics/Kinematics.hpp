#ifndef __KINEMATICS_HPP__
#define __KINEMATICS_HPP__

#include <Arduino.h>

typedef struct
{
    float per_pulse_distance;  // 每个脉冲轮子的前进距离
    int16_t motor_speed;       /* 电机速度 */
    int64_t last_encoder_tick; /* 上一次电机编码器读数 */
} motor_param_t;

typedef struct
{
    float x;            // x坐标
    float y;            // y坐标@ext:ms-vscode.cpptools Doxy
    float angle;        // 角度
    float linear_speed; // 线速度
    float angle_speed;  // 角速度
} odom_t;

class Kinematics
{
public:
    Kinematics() = default;
    ~Kinematics() = default;

    void set_motor_param(uint8_t id, float per_pulse_distance);
    void set_wheel_distance(float wheel_distance);
    void kinematics_forward(float left_speed, float right_speed, float &out_linear_speed, float &out_angle_speed);
    void kinematics_inverse(float linear_speed, float angle_speed, float &out_left_speed, float &out_right_speed);
    void update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick);
    int16_t get_motor_speed(uint8_t id);

    void update_odom(uint16_t dt);
    odom_t &get_odom();
    static void TransAngleInPI(float &angle, float &out_angle);

private:
    motor_param_t motor_param_[2];
    uint64_t last_update_time;
    float wheel_distance_;
    
    odom_t odom_;
};

#endif // __KINEMATICS_HPP__