import launch
import launch_ros

def generate_launch_description():
    # 生成 launch 描述
    action_node_detect_srv=launch_ros.actions.Node(
        package='chapt4_python_srv',
        executable='face_detect_node',
        name='face_detect_node',
        output='screen',    # 'screen' or 'log' or 'both'
    )
    action_node_detect_clt=launch_ros.actions.Node(
        package='chapt4_python_srv',
        executable='face_detect_client',
        name='face_detect_client',
        output='screen',
    )
    return launch.LaunchDescription([
        # 执行 action 动作
        action_node_detect_srv,
        action_node_detect_clt,
    ])