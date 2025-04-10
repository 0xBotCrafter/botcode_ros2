#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;

public:
    TFListener() : Node("tf_listener")
    {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        timer_ = this->create_wall_timer(1s, std::bind(&TFListener::get_tf, this));
    }
    void get_tf()
    {
        try
        {
            const auto transform = buffer_->lookupTransform("base_link", "target_point", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            auto translation = transform.transform.translation;
            auto rotation = transform.transform.rotation;
            double yaw, pitch, roll;
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "Translation: x: %f, y: %f, z: %f", translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "Rotation: yaw: %f, pitch: %f, roll: %f", yaw, pitch, roll);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
