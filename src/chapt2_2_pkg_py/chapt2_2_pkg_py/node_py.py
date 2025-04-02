import rclpy # 导入ROS2客户端库
from rclpy.node import Node # 导入ROS2节点类

def main(args=None):
    rclpy.init(args=args) # 初始化ROS2客户端库
    node = Node('node_py') # 创建ROS2节点
    node.get_logger().info('node_py has been created!') # 输出日志信息
    rclpy.spin(node)    # 运行节点
    rclpy.shutdown()    # 关闭ROS2客户端库