#ifndef __PIDCONTROLLER_HPP__
#define __PIDCONTROLLER_HPP__

class PidController
{
public:
    PidController() = default;                        // 默认构造函数
    PidController(float kp, float ki, float kd);      // 构造函数
    float update(float current);                      // 提供当前值，返回下次输出值
    void set_target(float target);                    // 更新目标值
    void set_kpid(float kp, float ki, float kd);      // 更新PID参数
    void reset();                                     // 重置PID
    void set_limit_out(float out_min, float out_max); // 设置限幅
private:
    float kp_;
    float ki_;
    float kd_;
    float target_;
    float out_min_;
    float out_max_;
    // pid
    float sum_error_;
    float derror_;
    float error_last_;             // 上次误差
    float sum_error_limit_ = 2500; // 积分限幅
};

#endif