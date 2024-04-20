from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    TextSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    AndSubstitution,
    NotSubstitution,
)
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    localization_launch_arg = DeclareLaunchArgument(
        "localization",
        default_value=TextSubstitution(text="false"),
        description="If true localization mode is launched",
    )
    mapping_launch_arg = DeclareLaunchArgument(
        "mapping",
        default_value=TextSubstitution(text="false"),
        description="If true mapping mode is launched (localization mode is dominant)",
    )
    map_file_launch_arg = DeclareLaunchArgument(
        "map_file",
        default_value=TextSubstitution(text="room_map"),
        description="If localization is used, the name of the map file used for localization.",
    )
    use_localization = LaunchConfiguration("localization")
    use_mapping = LaunchConfiguration("mapping")
    map_file = LaunchConfiguration("map_file")

    # if localization == True
    slam_node_localization = Node(
        parameters=[
            PathJoinSubstitution([FindPackageShare("slam_tutorial"), "config", "slam_params.yaml"]),
            {
                "map_file_name": PathJoinSubstitution([FindPackageShare("slam_tutorial"), "map", map_file]),
            },
        ],
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_node",
        remappings=[("pose", "slam_pose")],
        condition=IfCondition(use_localization),
    )

    # if mapping == True and localization == False
    slam_node_mapping = Node(
        parameters=[
            PathJoinSubstitution([FindPackageShare("slam_tutorial"), "config", "slam_params.yaml"]),
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_node",
        remappings=[("pose", "slam_pose")],
        condition=IfCondition(AndSubstitution(use_mapping, NotSubstitution(use_localization))),
    )

    # Publish a static transform between the `base_link` (robot) frame and the `laser` (Lidar) frame
    static_trans_laser_baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=(
            "--x 0.085 --y 0 --z 0 --roll 0 --pitch 0 --yaw 3.14 " + "--frame-id base_link --child-frame-id laser"
        ).split(" "),
    )

    return LaunchDescription(
        [
            localization_launch_arg,
            mapping_launch_arg,
            map_file_launch_arg,
            slam_node_localization,
            slam_node_mapping,
            static_trans_laser_baselink,
        ]
    )
