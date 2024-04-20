from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_tutorial"), "launch", "slam_launch.py"]
            )
        ),
        launch_arguments={
            "map_file_name": PathJoinSubstitution([FindPackageShare("slam_tutorial"), "map", map_file]),
            "localization": "true",
        }.items(),
    )

    # Launch rviz for the visualization
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("slam_tutorial"), "rviz", "slam.rviz"])],
    )

    # Launch a rosbag: see https://github.com/ros2/rosbag2?tab=readme-ov-file#using-in-launch on how to do this in a
    #  more efficient way for newer ROS versions.
    rosbag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            PathJoinSubstitution([FindPackageShare("slam_tutorial"), "bag", "sensor_data"]),
            "--clock",
        ],
        condition=IfCondition(using_bag_data),
    )

    static_trans_laser_baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=(
            "--x 0.085 --y 0 --z 0 --roll 0 --pitch 0 --yaw 3.14 " + "--frame-id base_link --child-frame-id laser"
        ).split(" "),
    )

    return LaunchDescription(
        [
            using_bag_data_launch_arg,
            map_file_launch_arg,
            do_use_sim_time,
            slam_launch,
            rviz,
            rosbag_player,
            static_trans_laser_baselink,
        ]
    )
