from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_package_path = get_package_share_path("project2")
    default_model_path = urdf_package_path / "urdf/bot.urdf.xacro"
    rplidar_pkg = get_package_share_path("rplidar_ros2")

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file",
    )
 
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )
    
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            str("configs/ekf.yaml"),
        ],
    )
    
    launch_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rplidar_pkg / "launch/rplidar_launch.py")
        ),
        launch_arguments={
            "frame_id": "lidar_link"
        }.items()
    )

    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher_node,
            robot_localization_node,
        ]
    )
