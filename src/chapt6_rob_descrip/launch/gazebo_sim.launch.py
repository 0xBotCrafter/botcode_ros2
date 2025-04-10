import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions


def generate_launch_description():
    # 1 初始化：获取路径、构造参数
    # 1.1 获取路径
    pkg_path = get_package_share_directory('chapt6_rob_descrip')
    default_xacro_path = pkg_path + '/urdf/fishbot/fishbot.urdf.xacro'
    rviz_config_path = pkg_path + '/config/rviz_robdisp.rviz'
    gazebo_world_path = pkg_path + '/worlds/custom_room.world'
    # 1.2 声明launch参数：'model'
    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_xacro_path),description='加载的模型文件路径'
    )
    # 1.3 获取urdf文件的内容,转为ParameterValue类型
    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    ros_description_value=launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)
    
    
    # 2.1 创建robot_state_publisher节点，传入urdf文件内容
    action_robot_state_publisher=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':ros_description_value}],
    )
    # 2.2 创建joint_state_publisher节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
        # # 多个robot的情况
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     namespace='robot1',
        #     parameters=[
        #         {'robot_description': robot1_urdf},
        #         {'frame_prefix': 'robot1/'}  # 关键：设置TF前缀
        #     ],
        #     remappings=[
        #         ('/joint_states', '/robot1/joint_states')  # 重映射关节话题
        #     ]
 
    # 3. 启动gazebo世界
    action_launch_gazebo=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch/gazebo.launch.py']
        ),
        launch_arguments=[('world',gazebo_world_path),('verbose','true')]
    )
    
    # action_launch_gazebo=launch.actions.ExecuteProcess(
    #     cmd=['gazebo','--verbose', gazebo_world_path,],
    #     output='screen'
    # )
    
    # 创建spawn_entity节点，传入xacro内容
    action_spawn_entity=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description','-entity','fishbot'],
    )
    
    # 加载joint_state_controller控制器
    action_load_joint_state_controller=launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_joint_state_broadcaster','--set-state', 'active'],
        output='screen'
    )
    
    # 加载effort_controller控制器
    action_load_effort_controller=launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_effort_controller','--set-state', 'active'],
        output='screen'
    ) 
     
    # 加载diff_drive_controller控制器
    action_load_diff_drive_controller=launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_diff_drive_controller','--set-state', 'active'],
        output='screen'
    )
    
    # 创建rviz2节点
    action_rviz2=launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
    )
        
        
    # 运行 actions
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        # action_joint_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
       
       launch.actions.RegisterEventHandler(
           event_handler=launch.event_handlers.OnProcessExit(
               target_action=action_spawn_entity,
               on_exit=[action_load_joint_state_controller],
           )
       ),
       
       launch.actions.RegisterEventHandler(
           event_handler=launch.event_handlers.OnProcessExit(
               target_action=action_load_joint_state_controller,
               on_exit=[
                        action_load_diff_drive_controller,
                        # action_load_effort_controller,
                        # action_rviz2,
                        ],
           )
       ),       
       
    ])