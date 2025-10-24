from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml

def load_yaml(pkg, file_path):
    with open(os.path.join(get_package_share_directory(pkg), file_path), 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("parol6_description"),
        "urdf", "parol6.urdf.xacro"
    )
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    srdf_path = os.path.join(
        get_package_share_directory("parol6_moveit2_config"),
        "config", "parol6.srdf"
    )
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics = load_yaml("parol6_moveit2_config", "config/kinematics.yaml")
    joint_limits = load_yaml("parol6_moveit2_config", "config/joint_limits.yaml")

    return LaunchDescription([
        Node(
            package="parol6_pipeline",
            executable="pick_controller_node",
            name="pick_controller",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics},
                {'robot_description_planning': joint_limits},
                {'use_sim_time': False}
            ]
        )
    ])