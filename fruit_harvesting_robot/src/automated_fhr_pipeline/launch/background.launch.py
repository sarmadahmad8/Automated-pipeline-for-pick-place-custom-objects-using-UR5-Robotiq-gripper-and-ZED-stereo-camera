from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    launch_ursim = LaunchConfiguration('launch_ursim')

    # Paths to included launch files
    ur_driver_launch = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    )
    moveit_launch = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot_ip', default_value='192.168.56.101'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('launch_ursim', default_value='false'),

        # Optionally launch URSim
        ExecuteProcess(
            cmd=['ros2', 'run', 'ur_client_library', 'start_ursim.sh', '-m', ur_type],
            condition=IfCondition(launch_ursim),
            output='screen'
        ),

        # Delay UR driver launch if URSim is launched
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(ur_driver_launch),
                    launch_arguments={'ur_type': ur_type, 'robot_ip': robot_ip}.items()
                )
            ],
            condition=IfCondition(launch_ursim)
        ),

        # Launch UR driver immediately if URSim is not launched
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_driver_launch),
            launch_arguments={'ur_type': ur_type, 'robot_ip': robot_ip}.items(),
            condition=UnlessCondition(launch_ursim)
        ),

        # Always include MoveIt launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={'ur_type': ur_type, 'launch_rviz': 'true'}.items()
        ),
    ])
