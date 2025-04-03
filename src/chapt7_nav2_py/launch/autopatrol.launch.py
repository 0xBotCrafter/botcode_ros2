import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取与拼接默认路径
    patrol_config_path = os.path.join(
        get_package_share_directory('chapt7_nav2_py'), 'config', 'patrol_config.yaml')
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
        package='chapt7_nav2_py',
        executable='patrol_node',
        parameters=[patrol_config_path]
    )
    ])