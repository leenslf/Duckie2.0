from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_teleop = LaunchConfiguration("use_teleop")
    world = PathJoinSubstitution(
        [FindPackageShare("duckiebot_gazebo"), "worlds", "duckie_world.world"]
    )
    gazebo_models = PathJoinSubstitution(
        [FindPackageShare("duckiebot_gazebo"), "models"]
    )
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare("duckiebot_description"), "urdf", "duckiebot.xacro"]
    )
    robot_description = ParameterValue(
        Command(["xacro ", robot_description_path]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_teleop",
                default_value="false",
                description="Launch teleop_twist_keyboard with the simulation.",
            ),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=[
                    gazebo_models,
                    ":",
                    EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"world": world}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "duckiebot",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.1",
                ],
                output="screen",
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop",
                condition=IfCondition(use_teleop),
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
