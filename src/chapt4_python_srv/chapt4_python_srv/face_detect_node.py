import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector # 导入自定义服务接口
import cv2 # 导入OpenCV库
import face_recognition # 导入face_recognition库
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # 导入cv_bridge库
import os # 导入os库
import time # 导入time库
from rcl_interfaces.msg import SetParametersResult 


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.bridge_ = CvBridge() # 创建cv_bridge对象
        self.declare_parameters( # 声明参数
            namespace='',
            parameters=[
                ('number_of_times_to_upsample', 1),
                ('model', 'hog')
            ]
        )
        self.number_of_times_to_upsample_ = self.get_parameter('number_of_times_to_upsample').value # 获取参数
        self.model_ = self.get_parameter('model').value # 获取模型参数
        
        self.default_image_path_ = os.path.join(get_package_share_directory('chapt4_python_srv'), 'resource/default.jpg') # 获取默认图片路径
        self.srv_ = self.create_service(FaceDetector, 'face_detect', self.face_detector_callback_) # 创建服务
        self.get_logger().info('Face Detector Service Ready')
        self.add_on_set_parameters_callback(self.parameters_callback) # 添加参数回调函数
        self.set_parameters([
            rclpy.Parameter('number_of_times_to_upsample', rclpy.Parameter.Type.INTEGER, 2),
            rclpy.Parameter('model', rclpy.Parameter.Type.STRING, 'hog')
                             ]) # 设置自身参数
        
    def parameters_callback(self, parameters): # 参数更新回调函数
        for parameter in parameters:
            if parameter.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample = parameter.value
            elif parameter.name == 'model':
                self.model = parameter.value
        return SetParametersResult(successful=True)

    def face_detector_callback_(self, request, response): # 服务回调函数
        if request.image.data:
            cv_image = self.bridge_.imgmsg_to_cv2(request.image) # 如果提供了图片数据，则将其转换为OpenCV格式
        else:
            cv_image = cv2.imread(self.default_image_path_) # 如果没有提供图片数据，则使用默认图片
            self.get_logger().info('No image data provided, using default image')
        
        start_time = time.time() # 记录开始时间
        self.get_logger().info('Start detecting faces...')
        
        # 检测人脸，记录结果信息
        face_locations = face_recognition.face_locations(cv_image,number_of_times_to_upsample=self.number_of_times_to_upsample_,model=self.model_) 
        response.use_time=time.time()-start_time # 记录检测时间
        response.number = len(face_locations) # 记录检测到的人脸数量
        for top,right,bottom,left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        
        self.get_logger().info(f'Detect end, use time: {response.use_time} s,number: {response.number}')
        return response # 返回响应

def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()