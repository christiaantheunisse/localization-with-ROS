# localization-with-ROS
This repository contains tutorials on the ROS2 packages `slam_toolbox` and `robot_localization`.

`slam_toolbox` is a ROS2 packages that can perform Simultaneous Localization And Mapping (SLAM) or just Localization on an earlier created map.

`robot_localization` is a ROS2 package that implements an Extended Kalman Filter (EKF), which is basically a Kalman Filter that uses a nonlinear model. It produces a filtered odometry estimate (position, orientation, velocity) based on different sensors.


## Installation

Download the repository in a workspace and build it. Make sure to use the same workspace location and name in order to be able to directly copy the commands in the tutorials below:

    mkdir -p ~/tutorial_ws/src
    cd ~/tutorial_ws/src
    git clone git@github.com:christiaantheunisse/localization-with-ROS.git .  # to install without project folder
    cd ..
    colcon build --packages-select slam_tutorial
    source install/setup.bash

Make sure to source your workspace (run `source ~/tutorial_ws/install/setup.bash`) in every new terminal window or add it to your `~/.bashrc`. Otherwise, ROS will give an error about the package not being found.

Download `slam_toolbox`:

    install command slam_toolbox

Download `robot_localizatoin`:

    install command robot_localizatoin

## SLAM

The [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) can either perform SLAM or just localization. The launch file `slam_launch.py`  launches a node that performs SLAM. To launch this file, run the following command:

    ros2 launch slam_tutorial slam_launch.py using_bag_data:=true


If every works correctly, you will see something like this in RViz2:

!!!!!!!! Add gif RVIZ2

The following command can be used to save the map and use it for localization later on. The relative path for the `filename` argument of the service starts in the root of the workspace, so providing only a name will store the map in `~/tutorial_ws`.

    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: src/slam_tutorial/map/my_first_map}"

As mentioned before, it is also possible to localize within an environment with an previously created map. Rebuild the package so ROS can find the just created map file. It is also possible to use an already available map which is called `room_map`.

    ros2 launch slam_tutorial localization_launch.py using_bag_data:=true map_file:=my_first_map

You might observe that the map still gets updated during localization, but this is intended behaviour as the slam_toolbox is intended to do lifelong mapping. 

### Explanation about the inner workings

According to the ROS [convention](https://www.ros.org/reps/rep-0105.html) ([simple explanation](https://robotics.stackexchange.com/questions/109759/confusion-on-tf2-frame-name-conventions-base-link-vs-odom)), the `slam_toolbox` is responsible for the global position update (no drift over time) and therefore publishes the `map -> odom` transform. So another node should publish the `odom -> base_link` transform, since the `base_link` frame is the fixed frame of the robot (the bag file contains this transform and publishes it on `/tf`). The `odom -> base_link` transform can be calculated based on the odometry (wheel encoders, IMU, etc.) which drift over time. So the `map -> base_link` transform defines the pose correct pose with respect to the real-world, but might contain jumps / be discontinouos. The `odom -> base_link` transform is continuous, but drifts over time, which is especially relevant for velocity and accelerations.

As far as I understand it, the `slam_toolbox` uses a SLAM algorithm that assumes that the updated pose should be in the vicinity of the previous pose, since it searches around the previous pose to find the updated pose. So a sudden 'jump' of the robot in the `map` frame (=real world), will violate this constraint and make the algorithm lose track of its pose from which it is not able to recover. These problem can in occur in one of the following situations:

- When the computational load is too high, the calculations will not be finished in time and the difference between two consecutive poses will big enough to violate the assumption.
- At start up, when the `map_start_pose` is too far of from the actual pose. This is only relevant in the localization mode.

#### For a deeper dive
A lot more information can be found on the [GitHub page](https://github.com/SteveMacenski/slam_toolbox) and in this [ROSCon Talk](https://vimeo.com/378682207). The GitHub page has a clear documentation of all the parameters and available services. Among other things, it is possible to...
- ...change the update rate and conditions.
- ...use it as a lifecycle node.
- ...set the initial pose through RVIZ's '2D Pose Estimate' or `/initialpose` topic.
- ...use an RViz plugin which can be used if you do a lot of mapping and saving of files.

## EKF

Setting up the Extended Kalman Filter (EKF) from the `robot_localization` packages is a more complicated task, but will significantly increase the localization performance compared to using the raw odometry together with the SLAM. This is not intended to be a complete guide on the [`robot_localization`](https://github.com/automaticaddison/robot_localization) packages, but the goal is to provide you with a basic understanding, some pointers and a working setup to experiment with. I hope this enables you find your way in the documentation and to setup the EKF on your own robot.



First, some information on the frames and the transforms.  tf etc. Current setup: raw odmetry and odom should provide base_link to odom tf.  slam_toolbox updates odom => map tf. Odom drifts over time, IMU also.

New situation

Odometry EKF

    ros2 launch ekf_tutorial odometry_ekf_launch.py  using_bag_data:=true

The purple oval shape gives the uncertainty on the position (covariance). The uncertainty should increase over time. However, the uncertainty on the odometry which is defined in the message is 0 (default value).

Navigation EKF

    ...

    # Non-holonomic robot example: https://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html
    # https://answers.ros.org/question/405263/prediction-step-and-imu-orientation-have-little-impact-on-robot_localization-position/
    # https://answers.ros.org/question/50870/what-frame-is-sensor_msgsimuorientation-relative-to/
    # Slides: https://roscon.ros.org/2015/presentations/robot_localization.pdf


## Exercise

To get your hands dirty you could try to combine the functionality from both packages. You should make a launch file that uses the bag file from the `slam_tutorial` package. Use the published scan messages to get a pose estimate with the `slam_toolbox`. Subsequently, apply two ekfs to get a filtered pose estimate.