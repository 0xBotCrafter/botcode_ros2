import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster 
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_broadcaster_ = TransformBroadcaster(self) 
        self.timer_ = self.create_timer(0.01, self.publish_tf)
        
    def publish_tf(self):
        transform_stamped = TransformStamped()
        transform_stamped.header.frame_id = 'camera_link'
        transform_stamped.child_frame_id = 'bottle'
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        
        transform_stamped.transform.translation.x = 0.2
        transform_stamped.transform.translation.y = 0.3
        transform_stamped.transform.translation.z = 0.5
        
        q = quaternion_from_euler(0, 0, 0)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]
        
        self.tf_broadcaster_.sendTransform(transform_stamped)
        self.get_logger().info('tf transform published')

def main():
    rclpy.init()
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()