import launch
import launch_ros

def generate_launch_description():
    # 声明launch参数
    action_declare_arg_background_g=launch.actions.DeclareLaunchArgument('launch_arg_g',default_value="150"),
    
    # 生成 launch 描述
    action_node_turtle_sim=launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        parameters=[{'background_g':launch.substitutions.LaunchConfiguration('launch_arg_g',default=150)}],
        output='screen',    # 'screen' or 'log' or 'both'
    )
    action_node_turtle_control=launch_ros.actions.Node(
        package='chapt4_cpp_srv',
        executable='turtle_control',
        name='turtle_control',
        output='screen',
    )
    action_node_turtle_patrol=launch_ros.actions.Node(
        package='chapt4_cpp_srv',
        executable='turtle_patrol',
        name='turtle_patrol',
        output='screen',
    )
    return launch.LaunchDescription([
        # 执行 action 动作
        action_node_turtle_sim,
        action_node_turtle_control,
        action_node_turtle_patrol,
    ])