from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    using_bag_data_launch_arg = DeclareLaunchArgument(
        "using_bag_data",
        default_value=TextSubstitution(text="false"),
        description="If bag data is used or not",
    )
    using_bag_data = LaunchConfiguration("using_bag_data")

    do_use_sim_time = SetParameter(name="use_sim_time", value=using_bag_data)

    velocity_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="velocity_ekf_node",
        parameters=[PathJoinSubstitution([FindPackageShare("ekf_tutorial"), "config", "ekf.yaml"])],
        remappings=[("odometry/filtered", "odometry/velocity_ekf")],
    )

    # Launch rviz for the visualization
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("ekf_tutorial"), "rviz", "velocity_ekf.rviz"])],
    )

    # Launch a rosbag: see https://github.com/ros2/rosbag2?tab=readme-ov-file#using-in-launch on how to do this in a
    #  more efficient way for newer ROS versions.
    rosbag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            PathJoinSubstitution([FindPackageShare("ekf_tutorial"), "bag", "ekf_data"]),
            "--clock",
        ],
        condition=IfCondition(using_bag_data),
    )

    static_trans_base_link_to_imu_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=(
            "--x 0.025 --y -0.038 --z 0 --roll 0 --pitch 0 --yaw -1.5708 --frame-id base_link --child-frame-id imu_link"
        ).split(" "),
    )

    return LaunchDescription(
        [
            using_bag_data_launch_arg,
            do_use_sim_time,
            velocity_ekf_node,
            rviz,
            rosbag_player,
            static_trans_base_link_to_imu_link,
        ]
    )
