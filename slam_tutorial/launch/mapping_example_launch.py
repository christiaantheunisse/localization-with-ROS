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
    using_bag_data = LaunchConfiguration("using_bag_data")

    # setting use_sim_time:=true requires to run the `ros bag play` with the option `--clock`
    do_use_sim_time = SetParameter(name="use_sim_time", value=using_bag_data)

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_tutorial"), "launch", "slam_launch.py"]
            )
        ),
        launch_arguments={
            "mapping": "true",
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

    return LaunchDescription(
        [
            using_bag_data_launch_arg,
            do_use_sim_time,
            slam_launch,
            rviz,
            rosbag_player,
        ]
    )
