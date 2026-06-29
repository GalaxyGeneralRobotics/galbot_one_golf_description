from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = "galbot_one_golf_description"


def launch_setup(context, *args, **kwargs):
    package_share = FindPackageShare(PACKAGE_NAME)
    rviz_config = PathJoinSubstitution([package_share, "config", "rviz", "model.rviz"])
    default_urdf_path = PathJoinSubstitution([package_share, "urdf", "galbot_one_golf.urdf"])

    use_gui = LaunchConfiguration("gui")
    urdf_path = LaunchConfiguration("urdf_path")
    urdf_file = urdf_path.perform(context) or default_urdf_path.perform(context)
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description_content = f.read()

    # The generated URDF stores portable relative mesh paths ("meshes/...").
    # RViz receives the URDF over the /robot_description topic and cannot
    # resolve relative paths reliably, so rewrite them to package:// URIs
    # that RViz resolves via ament_index (this is how the xacro sources
    # reference meshes and how RViz originally loaded them).
    robot_description_content = robot_description_content.replace(
        'filename="meshes/',
        f'filename="package://{PACKAGE_NAME}/meshes/',
    )
    robot_description = {"robot_description": robot_description_content}

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(use_gui),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare(PACKAGE_NAME),
                "urdf",
                "galbot_one_golf.urdf",
            ]),
            description="Absolute path to the URDF file to display.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Use joint_state_publisher_gui when true.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
