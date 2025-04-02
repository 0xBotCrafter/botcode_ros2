import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster # 导入静态tf广播器
from geometry_msgs.msg import TransformStamped # 导入坐标变换消息
from tf_transformations import quaternion_from_euler # 导入欧拉角转四元数函数
import math # 导入数学库

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self) # 创建静态tf广播器
        self.publish_static_tf() # 发布静态tf
        
    def publish_static_tf(self):
        transform_stamped = TransformStamped() # 创建坐标变换消息
        
        transform_stamped.header.frame_id = 'base_link' # 设置父坐标系
        transform_stamped.child_frame_id = 'camera_link' # 设置子坐标系
        transform_stamped.header.stamp = self.get_clock().now().to_msg() # 设置时间戳
        
        # 设置坐标变换
        transform_stamped.transform.translation.x = 0.5
        transform_stamped.transform.translation.y = 0.3
        transform_stamped.transform.translation.z = 0.6
        
        # 设置旋转变换
        q = quaternion_from_euler(math.radians(180), 0, 0)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]
        
        self.static_broadcaster_.sendTransform(transform_stamped) # 发布静态tf
        self.get_logger().info('Static transform published')

def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()