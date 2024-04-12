# Interbotix bug fixing
SO1E05, 2024.04.12

# 1. Objective

In this episode, we shared our fun experience to fix a bug in interbotix xsarm puppet package. 

Last week, we achieved partial success in controlling Interbotix robotic arms manufactured by Trossen through their APIs.

While we were able to successfully control each robotic arm individually, [the interbotix xsarm puppet package](https://www.youtube.com/watch?v=DnjbNXxBE_8) at 7:53, failed.

To be more specific, the two robotic arms were the Interbotix wx250s, which we used as the leader, and the vx300s, which we used as the follower. 

We manually controlled the wx250s leader robotic arm, and expected the vx300s follower robotic arm to automatically imitate the exact same movements.However, the vx300s follower robotic arm didn't imitate the exact same movements as expected.

[![S01E05 Interbotix robotic arm bug fixing](https://img.youtube.com/vi/00pYmQL-r34/hqdefault.jpg)](https://www.youtube.com/watch?v=00pYmQL-r34)


# 2. Solution

TL;DR

The solution is simple, 

## Step 1. Download the updated xsarm_puppet.launch.py, in addition to the package.xml and CMakeLists.txt.

1. Download xsarm_puppet.launch.py

You can download the updated xsarm_puppet.launch.py, either from the [interbotix github repo](https://github.com/Interbotix/interbotix_ros_manipulators/blob/b370dc0bd451f90429e56e597e06cd07dc0435b6/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/launch/xsarm_puppet.launch.py), or, for convenience, from [this repo](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/launch/xsarm_puppet.launch.py). 

2. Download package.xml

If your compilation of the puppet package failed, you might need to update your package.xml. 

You can download the package.xml, either from the [interbotix github repo](https://github.com/Interbotix/interbotix_ros_manipulators/blob/b370dc0bd451f90429e56e597e06cd07dc0435b6/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/package.xml), or, for convenience, from [this repo](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/package.xml). 

3. Download CMakeLists.txt

If your compilation of the puppet package failed, you might also need to update your CMakeLists.txt. 

You can download the CMakeLists.txt, either from the [interbotix github repo](https://github.com/Interbotix/interbotix_ros_manipulators/blob/b370dc0bd451f90429e56e597e06cd07dc0435b6/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/CMakeLists.txt), or, for convenience, from [this repo](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/CMakeLists.txt). 



## Step 2. Recompile the interbotix system. 

Use the downloaded codes, to replace your old codes.

Then recompile the interbotix system, by executing the following commands. 

~~~
$ cd ~/interbotix_ws
$ rm -rf build/ install/ log/
$ colcon build --symlink-install
$ source /home/robot/interbotix_ws/install/setup.bash 
~~~

## Step 3. Rerun the xsarm puppet package

~~~
$ ros2 launch interbotix_xsarm_puppet xsarm_puppet.launch3.py robot_model_leader:=wx250s robot_model_follower:=vx300s

[INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-04-09-09-25-35-020802-pc-153345
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [xs_sdk-1]: process started with pid [153367]
[INFO] [robot_state_publisher-2]: process started with pid [153369]
[INFO] [xs_sdk-3]: process started with pid [153371]
[INFO] [robot_state_publisher-4]: process started with pid [153373]
[INFO] [xsarm_puppet-5]: process started with pid [153375]
[INFO] [static_transform_publisher-6]: process started with pid [153377]
[INFO] [static_transform_publisher-7]: process started with pid [153379]
[INFO] [rviz2-8]: process started with pid [153381]
[xs_sdk-1] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-1] [INFO] Using logging level 'INFO'.
[xs_sdk-1] [INFO] Loaded mode configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/leader_modes.yaml'.
[xs_sdk-1] [INFO] Loaded motor configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx250s.yaml'.
[xs_sdk-3] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-3] [INFO] Using logging level 'INFO'.
[xs_sdk-1] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[xs_sdk-3] [INFO] Loaded mode configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/follower_modes.yaml'.
[xs_sdk-3] [INFO] Loaded motor configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/vx300s.yaml'.
[xs_sdk-3] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[static_transform_publisher-6] [INFO] [1712625935.491035962] [tf_broadcaster_leader]: Spinning until stopped - publishing transform
[static_transform_publisher-6] translation: ('0.000000', '-0.250000', '0.000000')
[static_transform_publisher-6] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-6] from '/world' to 'leader/base_link'
[static_transform_publisher-7] [INFO] [1712625935.493756687] [tf_broadcaster_follower]: Spinning until stopped - publishing transform
[static_transform_publisher-7] translation: ('0.000000', '0.250000', '0.000000')
[static_transform_publisher-7] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-7] from '/world' to 'follower/base_link'
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  8, Model: 'XL430-W250', Joint Name: 'wrist_rotate'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  8, Model: 'XM430-W350', Joint Name: 'wrist_rotate'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  7, Model: 'XM430-W350', Joint Name: 'wrist_angle'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  7, Model: 'XM540-W270', Joint Name: 'wrist_angle'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  5, Model: 'XM430-W350', Joint Name: 'elbow_shadow'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  5, Model: 'XM540-W270', Joint Name: 'elbow_shadow'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  4, Model: 'XM430-W350', Joint Name: 'elbow'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  4, Model: 'XM540-W270', Joint Name: 'elbow'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  6, Model: 'XM430-W350', Joint Name: 'forearm_roll'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  6, Model: 'XM540-W270', Joint Name: 'forearm_roll'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  2, Model: 'XM430-W350', Joint Name: 'shoulder'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  2, Model: 'XM540-W270', Joint Name: 'shoulder'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  9, Model: 'XL430-W250', Joint Name: 'gripper'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  9, Model: 'XM430-W350', Joint Name: 'gripper'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  3, Model: 'XM430-W350', Joint Name: 'shoulder_shadow'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  3, Model: 'XM540-W270', Joint Name: 'shoulder_shadow'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  1, Model: 'XM430-W350', Joint Name: 'waist'.
[xs_sdk-3] [INFO]         Found DYNAMIXEL ID:  1, Model: 'XM540-W270', Joint Name: 'waist'.
[xs_sdk-1] [WARN] Writing startup register values to EEPROM. This only needs to be done once on a robot if using a default motor config file, or after a motor config file has been modified. Can set `write_eeprom_on_startup` to false from now on.
[xs_sdk-3] [WARN] Writing startup register values to EEPROM. This only needs to be done once on a robot if using a default motor config file, or after a motor config file has been modified. Can set `write_eeprom_on_startup` to false from now on.
[xs_sdk-1] [INFO] The operating mode for the 'arm' group was changed to 'position' with profile type 'velocity'.
[xs_sdk-3] [INFO] The operating mode for the 'arm' group was changed to 'position' with profile type 'velocity'.
[xs_sdk-3] [INFO] The operating mode for the 'gripper' joint was changed to 'linear_position' with profile type 'velocity'.
[xs_sdk-3] [INFO] Interbotix X-Series Driver is up!
[xs_sdk-1] [INFO] The 'arm' group was torqued off.
[xs_sdk-1] [INFO] The operating mode for the 'gripper' joint was changed to 'position' with profile type 'velocity'.
[xs_sdk-1] [INFO] The 'gripper' joint was torqued off.
[xs_sdk-1] [INFO] Interbotix X-Series Driver is up!
[xs_sdk-3] [INFO] [1712625937.212520865] [interbotix_xs_sdk.xs_sdk]: InterbotixRobotXS is up!
[xs_sdk-1] [INFO] [1712625937.287749250] [interbotix_xs_sdk.xs_sdk]: InterbotixRobotXS is up!
[xsarm_puppet-5] [INFO] [1712625937.329807969] [xsarm_puppet]: Ready to start puppetting!
~~~


## Step 4. Play with the puppet arm

You can now manually controlled the wx250s leader robotic arm, and the vx300s follower robotic arm will automatically imitate the exact same movements, referring to the following video clip. 

[![Correct behavior of master/puppet arms](https://img.youtube.com/vi/fcGZVp7oBxo/hqdefault.jpg)](https://www.youtube.com/watch?v=fcGZVp7oBxo)


# 3. Phenomena and analysis

We first consulted with our friends who had successfully ran the interbotix puppet package. They told us that they ran the puppet package on ROS1 Neotic platform, but not on ROS2 Humble. After learning this, we also executed the interbotix puppet package on ROS1 Neotic and succeeded.

However, when we repeated the same task on ROS2 Humble, we failed. The vx300s follower robotic arm did not imitate the wx250s leader robotic arm and perform the exact same movements.

We further found that the wx250s leader robotic arm successfully transmitted the position parameters of the servos of each joint to the vx300s follower robotic arm. 

However, when the control center of the vx300s follower robotic arm gave correct instructions to the servos of each joint, the actual positions reached by the servos after executing the instructions showed an inexplicable reverse movement.

This is strange because if there exist an internal bug with vx300s arm, why did it success in ROS1, but fail in ROS2?

The details of the phenomena refers to [our bug report](https://github.com/orgs/Interbotix/discussions/48), posted in interbotix discussion forum. 

[The quick response from Soloman](https://github.com/orgs/Interbotix/discussions/48#discussioncomment-9049229), who is an engineer of interbotix, was very insightful when analyzing the strange phenomena, and very efficient to give a fixing solution. 

Then another interbotix engineer, [Luke followed up](https://github.com/orgs/Interbotix/discussions/48#discussioncomment-9060288). He modified all the related codes, and merged the updated code into interbotix product branch. 

By now, the obscure bug of xsarm puppet package has been completely fixed, and the discussion is quite fun. 