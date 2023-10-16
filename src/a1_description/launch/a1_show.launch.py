import os
import pathlib
import launch
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction

def generate_launch_description():

    # Set the path to this package.
    pkg_share = FindPackageShare(
        package='a1_description').find('a1_description')

    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/unitree_a1_show.rviz')

    a1_description_urdf_path = os.path.join(
        pkg_share, 'urdf/a1.urdf')

    
   # print(a1_description_xacro_path)
    ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############
    # Launch configuration variables specific to simulation

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # Declare the launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time,}
        ],
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(a1_description_urdf_path).read(),
            'publish_frequency': 200.0,
        }],
    )


    start_rviz = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                LogInfo(msg='a1_robot_driver start'),
                start_rviz_cmd,
            ],
        )
    )

    return LaunchDescription([
        declare_rviz_config_file_cmd,
        robot_state_publisher,
        start_rviz,
    ])
