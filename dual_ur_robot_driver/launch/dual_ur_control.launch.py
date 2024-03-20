import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    ur_type = LaunchConfiguration('ur_type')
    left_robot_ip = LaunchConfiguration('left_robot_ip')
    left_controller_file = LaunchConfiguration('left_controller_file')
    left_tf_prefix = LaunchConfiguration('left_tf_prefix')
    left_use_fake_hardware = LaunchConfiguration('left_use_fake_hardware')
    left_launch_rviz = LaunchConfiguration('left_launch_rviz')
    left_script_command_port = LaunchConfiguration('left_script_command_port')
    left_trajectory_port = LaunchConfiguration('left_trajectory_port')
    left_reverse_port = LaunchConfiguration('left_reverse_port')
    left_script_sender_port = LaunchConfiguration('left_script_sender_port')

    right_robot_ip = LaunchConfiguration('right_robot_ip') 
    right_controller_file = LaunchConfiguration('right_controller_file')
    right_tf_prefix = LaunchConfiguration('right_tf_prefix')
    right_use_fake_hardware = LaunchConfiguration('right_use_fake_hardware')
    right_launch_rviz = LaunchConfiguration('right_launch_rviz')
    right_script_command_port = LaunchConfiguration('right_script_command_port')
    right_trajectory_port = LaunchConfiguration('right_trajectory_port')
    right_reverse_port = LaunchConfiguration('right_reverse_port')
    right_script_sender_port = LaunchConfiguration('right_script_sender_port')

    # # UR specific arguments
    ur_type_arg = DeclareLaunchArgument(
            "ur_type",
            default_value='ur3',
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
    )
    left_robot_ip_arg = DeclareLaunchArgument(
            "left_robot_ip",
            default_value='xxx.xxx.xxx.xxx',
            description="IP address by which the robot can be reached.",
    )
    left_controller_file_arg = DeclareLaunchArgument(
            "left_controller_file",
            default_value="left_ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
    )
    left_tf_prefix_arg = DeclareLaunchArgument(
            "left_tf_prefix",
            default_value="left_",
            description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
    )
    left_use_fake_hardware_arg = DeclareLaunchArgument(
            "left_use_fake_hardware",
            default_value="true",
    )
    left_launch_rviz_arg = DeclareLaunchArgument(
            "left_launch_rviz",
            default_value="false",
    )
    left_script_command_port_arg =  DeclareLaunchArgument(
            "left_script_command_port",
            default_value="50002",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    left_trajectory_port_arg = DeclareLaunchArgument(
            "left_trajectory_port",
            default_value="50003",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    left_reverse_port_arg = DeclareLaunchArgument(
            "left_reverse_port",
            default_value="50001",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    left_script_sender_port_arg = DeclareLaunchArgument(
            "left_script_sender_port",
            default_value="50005",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )


    right_robot_ip_arg = DeclareLaunchArgument(
            "right_robot_ip",
            default_value='yyy.yyy.yyy.yyy',
            description="IP address by which the robot can be reached.",
    )
    right_controller_file_arg = DeclareLaunchArgument(
            "right_controller_file",
            default_value="right_ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
    )
    right_tf_prefix_arg = DeclareLaunchArgument(
            "right_tf_prefix",
            default_value="right_",
            description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
    )
    right_use_fake_hardware_arg = DeclareLaunchArgument(
            "right_use_fake_hardware",
            default_value="true",
    )
    right_launch_rviz_arg = DeclareLaunchArgument(
            "right_launch_rviz",
            default_value="false",
    )
    right_script_command_port_arg =  DeclareLaunchArgument(
            "right_script_command_port",
            default_value="50010",
            description="Port that will be opened to forward script commands from the driver to the robot",
    )

    right_trajectory_port_arg = DeclareLaunchArgument(
            "right_trajectory_port",
            default_value="50009",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    right_reverse_port_arg = DeclareLaunchArgument(
            "right_reverse_port",
            default_value="50006",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    right_script_sender_port_arg = DeclareLaunchArgument(
            "right_script_sender_port",
            default_value="50007",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )


    ur_launch_dir = get_package_share_directory('dual_ur_robot_driver')

    left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_launch_dir, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': ur_type,
                          'robot_ip': left_robot_ip,
                          'controllers_file': left_controller_file,
                          'tf_prefix': left_tf_prefix,
                          'use_fake_hardware': left_use_fake_hardware,
                          'launch_rviz': left_launch_rviz,
                          'script_command_port': left_script_command_port,
                          'trajectory_port': left_trajectory_port,
                          'reverse_port': left_reverse_port,
                          'script_sender_port': left_script_sender_port,
                          }.items())
    
    left_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('left'),
         left
      ]
     
    )

    right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_launch_dir, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': ur_type,
                          'robot_ip': right_robot_ip,
                          'controllers_file': right_controller_file,
                          'tf_prefix': right_tf_prefix,
                          'use_fake_hardware': right_use_fake_hardware,
                          'launch_rviz': right_launch_rviz,
                          'script_command_port': right_script_command_port,
                          'trajectory_port': right_trajectory_port,
                          'reverse_port': right_reverse_port,
                          'script_sender_port': right_script_sender_port,
                          }.items())
    
    right_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('right'),
         right
      ]
    )
    
    return LaunchDescription([
        ur_type_arg,
        left_robot_ip_arg,
        left_controller_file_arg,
        left_tf_prefix_arg,
        left_use_fake_hardware_arg,
        left_launch_rviz_arg,
        left_script_command_port_arg,
        left_trajectory_port_arg,
        left_reverse_port_arg,
        left_script_sender_port_arg,
        right_robot_ip_arg,
        right_controller_file_arg,
        right_tf_prefix_arg,
        right_use_fake_hardware_arg,
        right_launch_rviz_arg,
        right_script_command_port_arg,
        right_trajectory_port_arg,
        right_reverse_port_arg,
        right_script_sender_port_arg,
        left_with_namespace,
        right_with_namespace
    ])