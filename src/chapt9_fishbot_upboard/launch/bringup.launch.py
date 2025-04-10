import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory(
        'chapt9_fishbot_upboard')
    ydlidar_drive_dir = get_package_share_directory(
        'ydlidar')

    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_dir, '/launch', '/urdf2tf.launch.py']),
    )

    odom2tf = launch_ros.actions.Node(
        package='chapt9_fishbot_upboard',
        executable='odom2tf',
        output='screen'
    )

    microros_agent = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4','--port','8888'],
        output='screen'
    )

    wifi2serial =  launch_ros.actions.Node(
        package='chapt9_wifi2serial',
        executable='tcp_server',
        parameters=[{'serial_port': '/tmp/tty_laser'}],
        output='screen'
    )

    ydlidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ydlidar_drive_dir, '/launch', '/ydlidar_launch.py']),
    )

    # 使用 TimerAction
    wifi2serial_delay = launch.actions.TimerAction(period=6.0, actions=[wifi2serial])
    ydlidar_delay = launch.actions.TimerAction(period=8.0, actions=[ydlidar])
    return launch.LaunchDescription([
        urdf2tf,
        odom2tf,
        microros_agent,
        wifi2serial_delay,
        ydlidar_delay,
    ])