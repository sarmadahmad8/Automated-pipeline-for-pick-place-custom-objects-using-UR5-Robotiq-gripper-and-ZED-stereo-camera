from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_manual_detection = LaunchConfiguration('use_manual_detection')

    return LaunchDescription([
        # Declare again because it's used here too
        DeclareLaunchArgument('use_manual_detection', default_value='true'),

        # Robotiq 2F gripper adapter node
        Node(
            package='robotiq_2f_urcap_adapter',
            executable='robotiq_2f_adapter_node.py',
            name='robotiq_adapter',
            output='screen',
            parameters=[{'robot_ip': '192.168.1.102'}]
        ),

        # Conditional detection node
        Node(
            package='detection_publishers',
            executable='manual_detection_publisher',
            name='detection_node',
            condition=IfCondition(use_manual_detection),
            output='screen'
        ),
        Node(
            package='detection_publishers',
            executable='automatic_detection_publisher',
            name='detection_node',
            condition=UnlessCondition(use_manual_detection),
            output='screen'
        ),

        # Move group receiver node
        Node(
            package='detection_recievers',
            executable='detection_reciever',
            name='detection_reciever',
            output='screen',
            parameters=[
                '/home/sarmadahmad8/workspace/ros_ur_driver/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml'
            ]
        ),
    ])
