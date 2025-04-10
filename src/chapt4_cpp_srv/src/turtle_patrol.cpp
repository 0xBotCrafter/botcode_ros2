#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol; // 定义一个别名
using SetParameters = rcl_interfaces::srv::SetParameters;

class TurtlePatrol : public rclcpp::Node // 继承rclcpp::Node类
{
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_; // 定义一个客户端指针
    rclcpp::TimerBase::SharedPtr timer_;       // 定义一个定时器指针

public:
    TurtlePatrol(const std::string &node_name) : Node(node_name) // 构造函数
    {
        srand(time(NULL));                                       // 设置随机数种子
        patrol_ = this->create_client<Patrol>("patrol");         // 创建一个客户端
        timer_ = this->create_wall_timer(5000ms, [&]() -> void { // 创建一个定时器
            // 1. 检查srv是否上线
            while (!this->patrol_->wait_for_service(1s)) // 等待1s
            {
                if (!rclcpp::ok()) // 如果程序退出
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the patrol service. Exiting.");
                    return;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "patrol service not available, waiting again...");
                }
            }
            // 2. 产生request
            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand() % 13;
            request->target_y = rand() % 13;
            RCLCPP_INFO(this->get_logger(), "Request: target_x = %f, target_y = %f", request->target_x, request->target_y); // 打印request

            // 3. 调用客户端
            this->patrol_->async_send_request(request, [&](rclcpp::Client<Patrol>::SharedFuture future) -> void
                                              {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: result = %d", response->result); });
        });
    };

    // 调用函数,更新参数
    void update_parameters_k(double k){
        // 1 创建参数对象
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.double_value = k;
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value = param_value;

        // 2 创建客户端，调用set_parameters服务
        auto set_param = this->create_client<SetParameters>("/turtle_control/set_parameters");
        // 2.1 检查srv是否上线
        while (!set_param->wait_for_service(1s)){ // 等待1s
            if (!rclcpp::ok()){ // 如果程序退出
                RCLCPP_INFO(this->get_logger(), "update parameters failed1");
                return;
            }
            else{
                RCLCPP_INFO(this->get_logger(), "patrol service not available, waiting again...");
            }
        }
        // 2.2 产生request
        auto request = std::make_shared<SetParameters::Request>();
        request->parameters.push_back(param);
        // request->parameter.push_back(param2);
        // 2.3 调用客户端设置参数
        auto future = set_param->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        
        // 3 检查设置结果
        for (auto &result : response->results){
            if (result.successful == 0){
                RCLCPP_ERROR(this->get_logger(), "update parameters failed2");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "update parameters successful");
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlePatrol>("turtle_patrol");
    node->update_parameters_k(0.6);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
