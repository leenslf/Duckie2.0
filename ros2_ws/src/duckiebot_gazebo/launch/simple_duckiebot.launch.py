from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    duckiebot_description_share = FindPackageShare("duckiebot_description")

    xacro_file = PathJoinSubstitution(
        [duckiebot_description_share, "urdf", "simple_duckiebot.xacro"]
    )
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    teleop = LaunchConfiguration("teleop")

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.1"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("teleop", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_simple_duckiebot",
                output="screen",
                arguments=[
                    "-entity",
                    "simple_duckiebot",
                    "-topic",
                    "robot_description",
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    z,
                    "-R",
                    roll,
                    "-P",
                    pitch,
                    "-Y",
                    yaw,
                ],
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                prefix="xterm -e",
                condition=IfCondition(teleop),
            ),
        ]
    )
