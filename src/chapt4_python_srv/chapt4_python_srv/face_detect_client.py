import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector # 导入自定义服务接口
import cv2 # 导入OpenCV库
import face_recognition # 导入face_recognition库
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # 导入cv_bridge库
import os # 导入os库
import time # 导入time库
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class FaceDetectClient(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.client_ = self.create_client(FaceDetector, 'face_detect') # 创建客户端
        self.bridge_ = CvBridge() # 创建cv_bridge对象
        self.number_of_times_to_upsample = 1 # 上采样次数
        self.model_ = 'hog' # 人脸检测模型
        self.image_path_ = os.path.join(get_package_share_directory('chapt4_python_srv'), 'resource', 'pic1.jpg') # 图片路径
        self.image_ = cv2.imread(self.image_path_) # 读取默认图片
    
    def update_detect_model(self, model='hog'):# 设置服务端模型的参数
        # 1.创建参数对象
        param=Parameter()
        param.name='model'
        param_value=ParameterValue()
        param_value.string_value=model
        param_value.type=ParameterType.PARAMETER_STRING
        param.value=param_value
        
        # 2.创建参数列表
        parameters=[param]
        
        # 3.调用服务，更新参数
        # 3.1 等待服务可用
        update_param_client = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        while not update_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')    
        # 3.2 创建请求对象
        request = SetParameters.Request()
        request.parameters = parameters
        # 3.3 发送请求，等待响应
        future = update_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        # 4.处理服务响应结果
        for result in response.results:
            if result.successful:
                self.get_logger().info(f'Successfully updated model to {model}')
            else:
                self.get_logger().info('Failed to update model')
            

    def send_request(self):
        # 1.等待服务可用
        while not self.client_.wait_for_service(timeout_sec=1.0):# 等待服务可用
            self.get_logger().info('service not available, waiting again...')
        # 2.创建请求对象
        request = FaceDetector.Request() # 创建请求对象
        request.image = self.bridge_.cv2_to_imgmsg(self.image_)
        # 3.发送请求，等待响应
        future = self.client_.call_async(request) # 发送请求并返回未来对象
        def call_show_future(future): # 服务响应结果的回调函数
            response=future.result()
            for i in range(response.number):
                top = response.top[i]
                right = response.right[i]
                bottom = response.bottom[i]
                left = response.left[i]
                cv2.rectangle(self.image_, (left, top), (right, bottom), (0, 255, 0), 2) # 在图片上绘制矩形框
            cv2.imshow('image', self.image_) # 显示图片
            cv2.waitKey(0) # 等待按键
        future.add_done_callback(call_show_future) # 添加回调函数
            
def main():
    rclpy.init()
    node = FaceDetectClient('face_detect_client')
    node.update_detect_model('hog')
    node.send_request()
    node.update_detect_model('cnn')
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()