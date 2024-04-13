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
    colcon build --packages-select 
    source install/setup.bash

Make sure the run `source install/setup.bash` in every new terminal window or add it to your `.bashrc`. Otherwise, ROS will through an error about the package not being found.

Download `slam_toolbox`:

    install command slam_toolbox

Download `robot_localizatoin`:

    install command robot_localizatoin

## SLAM

The [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) can either perform SLAM or just localization. The launch file `slam_launch.py`  launches a node that performs SLAM. To launch this file, run the following command:

    ros2 launch slam_tutorial slam_launch.py play_rosbag:=true

For the visualization, you need to run RViz2 (in a seperate terminal window). Use the provided configuration file to display the right topics by default. 

    rviz2 -d ~/tutorial_ws/src/slam_tutorial/rviz/slam.rviz

The bag data can be played using the command below (also in a seperate terminal window).

<!---
The `--clock` is very important, since it will tell the bag player to publish the time at the moment of recording the bag file to the `/clock` topic. When you run your nodes with `use_sim_time:=true`, they will use the time on this topic for the clock (i.e. when you call `self.get_clock()`/`this->get_clock()` in your node). Using this setup, the stamp in the header of your messages is comparable to the clock time, which might be important in some situation. Trust me, getting this right can save you a lot of debugging ;).
-->

    ros2 bag play ~/tutorial_ws/src/bag_files/sensor_data --clock

If every works correctly, you will see something like this in RViz2:

!!!!!!!! Add picture RVIZ2

The following commands can be used to save the map and use for localization later on. The relative path starts in the workspace, so providing only a name will store the map in `~/tutorial_ws`.

    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: src/slam_tutorial/map/my_first_map}"

Also possible to localize within this environment using the just created map. You might observe that the map still gets updated in some situations, but this is intended behaviour as the slam_toolbox is intended to do lifelong mapping (as far as I understand).


It is important to set the `start_map_pose` .. 

#### Explanation about the inner workings



#### For a deeper dive
- Possible to change the update rate / conditions
- Lifecycle node
- Initial position estimate to topic /startpose or with the rviz function ...
- There is a RViz plugin which can be used if you do a lot of mapping and saving of files
- More info about the description can be found on the [Github page](https://github.com/SteveMacenski/slam_toolbox) and in this [ROSCon Talk](https://vimeo.com/378682207).


## EKF

Setting up the EKF is a more complicated task, but will significantly increase the localization performance compared to using the raw odometry together with the SLAM. This is not intended to be a complete guide on the [`robot_localization`](https://github.com/automaticaddison/robot_localization) packages, but the goal is to provide you with a basic understanding and some pointers. I hope this enables you find your way in the documentation and to relatively setup your own ekf.

First, some information on frames and tf etc. Current setup: raw odmetry and odom should provide base_link to odom tf.  slam_toolbox updates odom => map tf. Odom drifts over time, IMU also.

New situation