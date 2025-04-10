#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

public:
    StaticTFBroadcaster() : Node("static_tf_broadcaster")
    {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_static_tf();
    }
    void publish_static_tf()
    {
        geometry_msgs::msg::TransformStamped static_transformStamped;

        static_transformStamped.header.stamp = this->get_clock()->now();
        static_transformStamped.header.frame_id = "map";
        static_transformStamped.child_frame_id = "target_point";
        static_transformStamped.transform.translation.x = 5.0;
        static_transformStamped.transform.translation.y = 3.0;
        static_transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 60 * M_PI / 180.0);
        static_transformStamped.transform.rotation = tf2::toMsg(q);
        this->static_tf_broadcaster_->sendTransform(static_transformStamped);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
