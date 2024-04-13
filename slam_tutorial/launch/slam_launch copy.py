from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    using_bag_data_launch_arg = DeclareLaunchArgument(
        "using_bag_data",
        default_value=TextSubstitution(text="false"),
        description="If bag data is used or not",
    )
    map_file_launch_arg = DeclareLaunchArgument(
        "map_file",
        default_value=TextSubstitution(text="room_map"),
        description="If applicable, the name of the map file used for localization.",
    )
    using_bag_data = LaunchConfiguration("using_bag_data")
    map_file = LaunchConfiguration("map_file")

    do_use_sim_time = SetParameter(name="use_sim_time", value=using_bag_data)

    slam_node = Node(
        parameters=[
            PathJoinSubstitution([FindPackageShare("slam_tutorial"), "config", "slam_params.yaml"]),
            {
                # "map_file_name": PathJoinSubstitution([FindPackageShare("slam_tutorial"), "map", map_file]),
                "use_lifecycle_manager": False,
            },
        ],
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_node",
        remappings=[("pose", "slam_pose")],
    )

    static_trans_laser_baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=("--x 0.085 --y 0 --z 0 --roll 0 --pitch 0 --yaw 3.14 " +
                   "--frame-id base_link --child-frame-id laser").split(" "),
    )

    return LaunchDescription([
        using_bag_data_launch_arg, # launch argument
        map_file_launch_arg,
        do_use_sim_time, # paramter
        slam_node,
        static_trans_laser_baselink,
    ])
