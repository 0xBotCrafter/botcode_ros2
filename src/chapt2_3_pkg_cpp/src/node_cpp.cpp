#include "rclcpp/rclcpp.hpp" // 包含ROS 2的头文件

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化ROS2客户端库
    auto node = std::make_shared<rclcpp::Node>("node_cpp"); // 创建节点
    RCLCPP_INFO(node->get_logger(), "node_cpp has been created!"); // 输出日志
    rclcpp::spin(node); // 阻塞运行节点，直到节点被关闭
    rclcpp::shutdown(); // 关闭ROS2客户端库
    return 0;
}