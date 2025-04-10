import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener,Buffer
from tf_transformations import euler_from_quaternion

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.buffer_,self)
        self.timer_ = self.create_timer(1.0,self.get_tf)
        
    def get_tf(self):
        try:
            result = self.buffer_.lookup_transform('base_link','bottle', rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0)) # 获取base_link到bottle的变换
            transform = result.transform # 获取变换信息
            self.get_logger().info(f'平移：{transform.translation},旋转:{transform.rotation}')
        except Exception as e:
            self.get_logger().warn(f'失败，原因：{str(e)}')
        
def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()