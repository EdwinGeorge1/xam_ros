#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    
    # ==============================
    # Robot MoveIt + Gazebo launch
    # ==============================
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                '_robot_moveit_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'dof': '6',
            'robot_type': 'lite',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
            # Pass through the add_realsense argument if needed
            'add_realsense_d435i': 'true', 
        }.items(),
    )
    
    # ==============================
    # ArUco Pose Detector Node
    # ==============================
    # aruco_pose_detector_node = Node(
    #     package='parol6_pipeline',
    #     executable='aruco_pose_detector',
    #     name='aruco_pose_detector',
    #     output='screen',
    #     parameters=[{
    #         'camera_frame': 'camera_color_optical_frame',
    #         'base_frame': 'link_base',
    #         'marker_id': 0,
    #         'marker_length': 0.05,
    #         'offset_roll': 0.0,
    #         'offset_pitch': -40.0,
    #         'offset_yaw': -90.0
    #     }]
    # )
    
    # ==============================
    # Pick Controller Node
    # ==============================
    # pick_controller_node = Node(
    #     package='parol6_pipeline',
    #     executable='pick_controller', # <-- CORRECTED NAME
    #     name='pick_controller',       # <-- Node name
    #     output='screen'
    # )
    
    return LaunchDescription([
        robot_moveit_gazebo_launch,
        # aruco_pose_detector_node,
        # pick_controller_node
    ])