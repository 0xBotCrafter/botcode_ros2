#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using SetParameterResult = rcl_interfaces::msg::SetParametersResult;
using Patrol = chapt4_interfaces::srv::Patrol; // 定义一个别名

class TurtleControlNode : public rclcpp::Node // 继承rclcpp::Node类
{
private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;    // 定义一个参数回调句柄
    rclcpp::Service<Patrol>::SharedPtr patrol_service_;                 // 定义一个服务指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 定义一个发布者指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;  // 定义一个订阅者指针
    double target_x_{1.0}, target_y_{1.0}, k_{0.5}, max_speed_{1.0};    // 定义目标位置和角度

public:
    TurtleControlNode(const std::string &node_name) : Node(node_name)   // 构造函数
    {
        this->declare_parameter("k", 1.0);// 声明参数
        this->declare_parameter("max_speed", 1.0);
        this->get_parameter("k", k_); // 获取参数
        this->get_parameter("max_speed", max_speed_); 
        this->set_parameter(rclcpp::Parameter("k", 2.0)); // 内部设置参数

        param_callback_handle_ = this->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> &parameters) -> SetParameterResult{
            SetParameterResult result;
            for (const auto &param : parameters){
                if (param.get_name() == "k")
                    k_ = param.as_double();
                else if (param.get_name() == "max_speed")
                    max_speed_ = param.as_double();

                RCLCPP_INFO(this->get_logger(), "Set parameter %s to %f", param.get_name().c_str(), param.as_double());
            }
            result.successful = true;
            return result;
        }); // 添加参数回调函数

        patrol_service_ = this->create_service<Patrol>("patrol", std::bind(&TurtleControlNode::patrol_callback_, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default); // patrol服务，回调函数为patrol_callback

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);                                                                            // turtle1/cmd_vel话题，消息类型为geometry_msgs::msg::Twist，队列长度为10
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleControlNode::on_pose_received_, this, std::placeholders::_1)); // turtle1/pose话题，消息类型为turtlesim::msg::Pose，队列长度为10，回调函数为pose_callback
    };

    void patrol_callback_(const std::shared_ptr<Patrol::Request> request, std::shared_ptr<Patrol::Response> response) // 服务回调函数
    {
        if ((request->target_x >= 0 && request->target_x < 12.0f) && (request->target_y >= 0 && request->target_y < 12.0f))
        {
            this->target_x_ = request->target_x;
            this->target_y_ = request->target_y;
            response->result = Patrol::Response::SUCCESS;
            RCLCPP_INFO(this->get_logger(), "Patrol service called with target position: (%f, %f)", request->target_x, request->target_y);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Invalid target position:targetX=%f,targetY=%f", request->target_x, request->target_y);
            response->result = Patrol::Response::FAIL;
        }
    }

    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose) // 订阅者回调函数,参数：收到数据的共享指针
    {
        // 1.获取当前位置
        auto current_x = pose->x;
        auto current_y = pose->y;
        auto current_theta = pose->theta;
        RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f)", current_x, current_y); // 输出当前位置

        // 2.计算距离和角度
        auto distance = sqrt(pow(target_x_ - current_x, 2) + pow(target_y_ - current_y, 2)); // 计算当前位置到目标位置的距离
        auto angle = atan2(target_y_ - current_y, target_x_ - current_x) - current_theta;    // 计算当前位置到目标位置的角度
        RCLCPP_INFO(this->get_logger(), "distance: %f, angle: %f", distance, angle);         // 输出距离和角度当前位置

        // 3.控制策略
        auto msg = geometry_msgs::msg::Twist(); // 创建一个geometry_msgs::msg::Twist类型的消息
        if (distance > 0.1){                    // 如果距离大于0.1
            if (fabs(angle) > 0.8) // 如果角度大于0.2
            {
                msg.angular.z = fabs(angle); // 设置角速度
            }
            else
            {
                msg.linear.x = k_ * distance; // 设置线速度
            }
        }

        // 4.限制最大速度
        if (msg.linear.x > max_speed_){
            msg.linear.x = max_speed_;
        }

        // 5.发布消息
        publisher_->publish(msg); // 发布消息
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                          // 初始化ROS2节点
    auto node = std::make_shared<TurtleControlNode>("turtle_control"); // 创建一个TurtleControlNode节点
    rclcpp::spin(node);                                                // 进入循环
    rclcpp::shutdown();                                                // 关闭ROS2节点
    return 0;
}
