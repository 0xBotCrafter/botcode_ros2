import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 动作1:启动其他launch   
    launch_path = [
        get_package_share_directory('chapt4_cpp_srv'),'/launch','/turtle_patrol.launch.py',
        # get_package_share_directory('chapt4_python_srv'),'launch','face_detect.launch.py',
        ]
    
    action_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch_path
        )
    )
    
    # 动作2:打印数据
    action_log_info = launch.actions.LogInfo(msg='launch file is running')
        
    # 动作3:执行一个进程(命令行)
    action_topic_list = launch.actions.ExecuteProcess(
        cmd=['ros2','topic', 'list'],
        output='screen'
    )
    
    # 动作4:动作成组
    action_group = launch.actions.GroupAction([
        # 动作5:定时器
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch.actions.LogInfo(msg='定时器执行中'),
                action_topic_list,
                ]
        ),
        launch.actions.TimerAction(
            period=6.0,
            actions=[
                action_launch,
                ]
        )
    ])
    
    return launch.LaunchDescription([
        # 执行 action 动作
        action_log_info,
        action_group,        
    ])