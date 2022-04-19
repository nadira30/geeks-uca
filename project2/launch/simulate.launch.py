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
    mapping_package_path = get_package_share_path("mapping_demo")
    slam_toolbox_package_path = get_package_share_path("slam_toolbox")
    default_model_path = urdf_package_path / "urdf/bot.urdf.xacro"
    default_rviz_config_path = urdf_package_path / "rviz/bot.rviz"
    world_path = gazebo_package_path / "worlds/demo_world.sdf"

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )
    model_arg = DeclareLaunchArgument(
        name="bot.urdf.xacro",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file",
    )
 
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(default_rviz_config_path),
        description="Absolute path to rviz config file",
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
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    gazebo_process = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            str(world_path),
        ],
        output="screen",
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "bot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.5",
        ],
        output="screen",
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            str(mapping_package_path / "configs/ekf.yaml"),
        ],
    )

    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_toolbox_package_path / "launch/online_async_launch.py")
        )
    )

    return LaunchDescription(
        [
            
            model_arg,
            rviz_arg,
            joint_state_publisher_node,
            gazebo_process,
            spawn_entity,
            robot_state_publisher_node,
            robot_localization_node,
            launch_slam,
            rviz_node,
        ]
    )
