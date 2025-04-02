#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <chapt3_status_interfaces/msg/system_status.hpp>

using SystemStatus = chapt3_status_interfaces::msg::SystemStatus;

class SystemStatusDisplay : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    std::shared_ptr<QLabel> label = std::make_shared<QLabel>();

    void subscriber_callback(const SystemStatus::SharedPtr system_status)
    {
        RCLCPP_INFO(this->get_logger(), "Received system status\n");
        std::stringstream str_show;
        str_show
            << "===================系统状态===================\n"
            << "数 据 时 间:\t" << system_status->stamp.sec << "\ts\n"
            << "主 机 名 称:\t" << system_status->host_name << "\t\n"
            << " CPU使用率：\t" << system_status->cpu_percent << "\t%\n"
            << "内存使用率 ：\t" << system_status->memory_percent << "\t%\n"
            << "内存总大小 : \t" << system_status->memory_total << "\tMB\n"
            << "内存剩余大小:\t" << system_status->memory_available << "\tMB\n"
            << "网络发送量 :\t" << system_status->net_sent << "\tMB\n"
            << "网络接收量 :\t" << system_status->net_recv << "\tMB\n"
            << "==============================================\n";

        label->setText(QString::fromStdString(str_show.str()));
    }

public:
    SystemStatusDisplay(const std::string &node_name) : Node(node_name)
    {
        subscriber_ = this->create_subscription<SystemStatus>(
            "sys_status", 10, std::bind(&SystemStatusDisplay::subscriber_callback, this, std::placeholders::_1));
        label->setText("Waiting for system status...");
        label->show();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SystemStatusDisplay>("system_status_display");
    std::thread thread([&]() -> void
                       { rclcpp::spin(node); });
    thread.detach(); // 分离线程
    app.exec();// 进入事件循环
    return 0;
}