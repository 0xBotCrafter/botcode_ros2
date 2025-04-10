#include <Arduino.h>
#include <Wire.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <PidController.hpp>
#include <Kinematics.hpp>
// 引入 micro-ROS 和 Wi-Fi 库
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>              // ROS2 消息类型
#include <nav_msgs/msg/odometry.h>                // ROS2 消息类型
#include <micro_ros_utilities/string_utilities.h> // 字符串内存分配初始化工具

// 声明机器人底层相关的结构体对象和变量
Esp32McpwmMotor motor;           // 创建一个数组用于存储两个电机
Esp32PcntEncoder encoders[2];    // 创建一个数组用于存储两个编码器
PidController pid_controller[2]; // 创建一个数组用于存储两个PID控制器
Kinematics kinematics;           // 创建一个机器人运动学对象

float target_linear_speed = 0.0; // 目标线速度 mm/s
float target_angle_speed = 0.0f; // 目标角速度 rad/s
float out_right_speed;
float out_left_speed;

// 声明ros2相关的对象和变量
rcl_allocator_t allocator;             // 内存分配器，动态内存分配管理
rclc_support_t support;                // 用于存储时钟、内存分配器和上下文
rclc_executor_t executor;              // 用于执行回调函数
rcl_node_t node;                       // ROS2 节点
rcl_subscription_t sub_cmd_vel;        // ROS2 订阅者
geometry_msgs__msg__Twist msg_cmd_vel; // ROS2 消息类型
rcl_publisher_t pub_odom;              // ROS2 发布者
nav_msgs__msg__Odometry msg_odom;      // ROS2 消息类型
rcl_timer_t timer;                     // ROS2 定时器

// 订阅者回调函数
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin; // 类型转换
  target_linear_speed = msg->linear.x * 1000;                                      // 转换为 mm/s
  target_angle_speed = msg->angular.z;                                             //  rad/s

  // 逆运动学计算
  kinematics.kinematics_inverse(target_linear_speed, target_angle_speed, out_left_speed, out_right_speed);
  pid_controller[0].set_target(out_left_speed);
  pid_controller[1].set_target(out_right_speed);
}

// 定时器回调函数:发布里程计
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  odom_t odom = kinematics.get_odom();                                         // 获取里程计
  int64_t stamp = rmw_uros_epoch_millis();                                     // 获取时间戳
  msg_odom.header.stamp.sec = static_cast<int32_t>(stamp / 1000);              // 秒
  msg_odom.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒
  msg_odom.pose.pose.position.x = odom.x;
  msg_odom.pose.pose.position.y = odom.y;
  msg_odom.pose.pose.orientation.w = cos(odom.angle * 0.5);
  msg_odom.pose.pose.orientation.x = 0;
  msg_odom.pose.pose.orientation.y = 0;
  msg_odom.pose.pose.orientation.z = sin(odom.angle * 0.5);
  msg_odom.twist.twist.linear.x = odom.linear_speed; // 线速度
  msg_odom.twist.twist.angular.z = odom.angle_speed; // 角速度
  // 发布里程计消息
  if (rcl_publish(&pub_odom, &msg_odom, NULL) != RCL_RET_OK)
  {
    Serial.println("Failed to publish msg_odom");
  }
}

// micro-ROS任务
void micro_ros_task(void *args)
{
  // 1.网络连接上位机：设置传输协议并延时等待设置完成
  IPAddress agent_ip;
  agent_ip.fromString("192.168.1.11"); // 主机ip
  char ssid[] = "CMCC-47g2";           // Wi-Fi SSID
  char pswd[] = "g9kptsX@";            // Wi-Fi 密码
  set_microros_wifi_transports(ssid, pswd, agent_ip, 8888);
  delay(2000);
  // 2.初始化内存分配器
  allocator = rcl_get_default_allocator();
  // 3.初始化support
  rclc_support_init(&support, 0, NULL, &allocator);
  // 4.初始化节点
  rclc_node_init_default(&node, "fishbot_motion_control", "", &support);
  // 5.初始化执行器
  unsigned int num_handles = 2; // 执行器程序的数量
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  // 6.初始化订阅者,并添加到执行器
  rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA);
  // 7.初始化发布者和定时器,并添加到执行器
  // 初始化msg_odom
  msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
  msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");
  // 初始化发布者和定时器
  rclc_publisher_init_best_effort(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
  rclc_executor_add_timer(&executor, &timer);
  // 8.时间同步
  while (!rmw_uros_epoch_synchronized())
  {
    rmw_uros_sync_session(1000);
    delay(10);
  }
  // 9.循环执行器
  rclc_executor_spin(&executor);
}

void setup()
{
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200
  // 初始化编码器
  encoders[0].init(0, 32, 33); // 初始化第一个编码器，使用GPIO 32和33连接
  encoders[1].init(1, 26, 25); // 初始化第二个编码器，使用GPIO 26和25连接
  // 初始化电机
  motor.attachMotor(0, 22, 23); // 初始化第一个电机，使用GPIO 连接
  motor.attachMotor(1, 12, 13); // 初始化第二个电机，使用GPIO 连接
  // 初始化PID控制器
  pid_controller[0].set_kpid(0.625, 0.125, 0.0);
  pid_controller[1].set_kpid(0.625, 0.125, 0.0);
  pid_controller[0].set_limit_out(-100, 100);
  pid_controller[1].set_limit_out(-100, 100);
  // 初始化轮子间距和电机参数
  kinematics.set_wheel_distance(175);       // 设置轮子间距 mm
  kinematics.set_motor_param(0, 0.1051566); // 设置第一个电机参数
  kinematics.set_motor_param(1, 0.1051566); // 设置第二个电机参数
  // 运动学逆解并设置速度
  kinematics.kinematics_inverse(target_linear_speed, target_angle_speed, out_left_speed, out_right_speed);
  pid_controller[0].set_target(out_left_speed);
  pid_controller[1].set_target(out_right_speed);

  // 创建任务运行 micro-ros-task
  xTaskCreate(micro_ros_task, // 任务函数
              "micro_ros",    // 任务名称
              10240,          // 任务堆栈大小
              NULL,           // 传递给任务函数的参数
              1,              // 任务优先级
              NULL);          // 任务句柄
}

void loop()
{
  delay(10);                                                                                                              // 等待10毫秒
  kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());                                // 更新电机速度和编码器数据
  motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0)));                                     // 更新第一个电机速度
  motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1)));                                     // 更新第二个电机速度
  Serial.printf("left speed: %f, right speed: %f\n", kinematics.get_motor_speed(0), kinematics.get_motor_speed(1));       // 输出左右轮速度
  Serial.printf("x=%f, y=%f, angle=%f\n", kinematics.get_odom().x, kinematics.get_odom().y, kinematics.get_odom().angle); // 输出机器人位置
  Serial.printf("time=%d, left ticks=%d, right ticks=%d\n", millis(), encoders[0].getTicks(), encoders[1].getTicks());    // 输出时间和编码器数据
}
