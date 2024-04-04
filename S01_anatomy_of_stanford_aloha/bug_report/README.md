# Bug report on xsarm puppet example with ROS2 humble

Following the instruction on xsarm puppet ros package on youtube, [Interbotix Tutorials: X-Series Arms | Working With Multiple Arms](https://www.youtube.com/watch?v=DnjbNXxBE_8) at 7:53, we successfully ran the puppet example with ROS1 noetic, but failed with ROS2 humble. Can you please give us a hint how to fix the problem with ROS2?

More details, 

We used a wx250s for the master arm, and a vx300s for the puppet arm, to run xsarm puppet example. 

## 1. Ran xarms puppet example on ROS2 humble, failed
   
We downloaded and installed interbotix ROS2 version, then launched xsarm_puppet package using this command,
~~~
$ cd ~/interbotix_ws
$ ros2 launch interbotix_xsarm_puppet xsarm_puppet.launch.py robot_model_leader:=wx250s robot_model_follower:=vx300s

[INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-04-03-23-56-25-004170-pc-470355
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [xs_sdk-1]: process started with pid [470378]
[INFO] [robot_state_publisher-2]: process started with pid [470380]
[INFO] [xs_sdk-3]: process started with pid [470382]
[INFO] [robot_state_publisher-4]: process started with pid [470384]
[INFO] [xsarm_puppet-5]: process started with pid [470386]
[INFO] [static_transform_publisher-6]: process started with pid [470388]
[INFO] [static_transform_publisher-7]: process started with pid [470390]
[INFO] [rviz2-8]: process started with pid [470392]
[xs_sdk-1] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-1] [INFO] Using logging level 'INFO'.
[xs_sdk-1] [INFO] Loaded mode configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/leader_modes.yaml'.
[xs_sdk-3] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-3] [INFO] Using logging level 'INFO'.
[xs_sdk-1] [INFO] Loaded motor configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx250s.yaml'.
[xs_sdk-3] [INFO] Loaded mode configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/follower_modes.yaml'.
[xs_sdk-3] [INFO] Loaded motor configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx250s.yaml'.
[xs_sdk-1] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[xs_sdk-3] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[static_transform_publisher-6] [INFO] [1712159785.478042512] [tf_broadcaster_leader]: Spinning until stopped - publishing transform
[static_transform_publisher-6] translation: ('0.000000', '-0.250000', '0.000000')
[static_transform_publisher-6] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-6] from '/world' to 'leader/base_link'
[static_transform_publisher-7] [INFO] [1712159785.481227766] [tf_broadcaster_follower]: Spinning until stopped - publishing transform
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
[xs_sdk-1] [INFO] The operating mode for the 'arm' group was changed to 'position' with profile type 'time'.
[xs_sdk-3] [INFO] The operating mode for the 'arm' group was changed to 'position' with profile type 'time'.
[xs_sdk-1] [INFO] The 'arm' group was torqued off.
[xs_sdk-3] [INFO] The operating mode for the 'gripper' joint was changed to 'linear_position' with profile type 'time'.
[xs_sdk-3] [INFO] Interbotix X-Series Driver is up!
[xs_sdk-1] [INFO] The operating mode for the 'gripper' joint was changed to 'pwm' with profile type 'time'.
[xs_sdk-1] [INFO] The 'gripper' joint was torqued off.
[xs_sdk-1] [INFO] Interbotix X-Series Driver is up!
[xs_sdk-3] [INFO] [1712159787.295818931] [interbotix_xs_sdk.xs_sdk]: InterbotixRobotXS is up!
[xs_sdk-1] [INFO] [1712159787.350688980] [interbotix_xs_sdk.xs_sdk]: InterbotixRobotXS is up!
[xsarm_puppet-5] [INFO] [1712159787.393662517] [xsarm_puppet]: Ready to start puppetting!
[INFO] [rviz2-8]: process has finished cleanly [pid 470392]
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[static_transform_publisher-6] [INFO] [1712159865.226433126] [rclcpp]: signal_handler(signum=2)
[static_transform_publisher-7] [INFO] [1712159865.226501054] [rclcpp]: signal_handler(signum=2)
[xs_sdk-1] [INFO] [1712159865.226601824] [rclcpp]: signal_handler(signum=2)
[xsarm_puppet-5] [INFO] [1712159865.226602378] [rclcpp]: signal_handler(signum=2)
[xs_sdk-3] [INFO] [1712159865.226602333] [rclcpp]: signal_handler(signum=2)
[INFO] [static_transform_publisher-7]: process has finished cleanly [pid 470390]
[INFO] [static_transform_publisher-6]: process has finished cleanly [pid 470388]
[INFO] [xsarm_puppet-5]: process has finished cleanly [pid 470386]
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 470384]
[INFO] [robot_state_publisher-2]: process has finished cleanly [pid 470380]
[INFO] [xs_sdk-1]: process has finished cleanly [pid 470378]
[INFO] [xs_sdk-3]: process has finished cleanly [pid 470382]
~~~

It failed, with the following details, 

1. After launching the interbotix_xsarm_puppet package as above, the leader arm wx250s, started with a home/sleep position in rviz, as we expected. 
  However, the follower arm vx300s, stretched out in stead of staying at home/sleep position, referring to the following picture.
  ![The follower arm vx300s stretched out unexpectedly](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/image/ros2_humble_starting_position.png)


2. When we manually manipulated the leader arm wx250s, the follower arm vx300s didn't exactly imitate the movement of its leader, referring to [this video clip](https://www.youtube.com/watch?v=Y_bUKOU6P9g).

  [![The follower arm vx300s didn't exactly imitate the movement of its leader wx250s](https://img.youtube.com/vi/Y_bUKOU6P9g/hqdefault.jpg)](https://www.youtube.com/watch?v=Y_bUKOU6P9g)

 
3. All the source codes, including the configuration files, are stored in [our github repo](https://github.com/housework-robot/main/tree/main/S01_anatomy_of_stanford_aloha/bug_report/ros2_humble).


## 2. Ran xarms puppet example on ROS1 noetic, successful

We downloaded and installed interbotix ROS1 version, then launched xsarm_puppet package using this command,

~~~
$ cd ~/interbotix_ws
$ roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s robot_model2:=vx300s

... logging to /home/robot/.ros/log/8303e2fa-f1c8-11ee-b780-7d6fab270688/roslaunch-robot-31260.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.
started roslaunch server http://192.168.0.100:37623/
SUMMARY
========
PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.16.0
 * /wx250s/robot_description: <?xml version="1....
 * /wx250s/xs_sdk/load_configs: True
 * /wx250s/xs_sdk/mode_configs: /home/robot/inter...
 * /wx250s/xs_sdk/motor_configs: /home/robot/inter...
NODES
  /wx250s/
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rviz (rviz/rviz)
    xs_sdk (interbotix_xs_sdk/xs_sdk)
 
ROS_MASTER_URI=http://localhost:11311
process[wx250s/robot_state_publisher-1]: started with pid [31339]
process[wx250s/rviz-2]: started with pid [31340]
process[wx250s/xs_sdk-3]: started with pid [31341]
[ INFO] [1712162326.428827007]: [xs_sdk] Loaded mode configs from '/home/robot/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/modes.yaml'.
[ INFO] [1712162326.429136415]: [xs_sdk] Loaded motor configs from '/home/robot/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/wx250s.yaml'.
[ INFO] [1712162326.430153435]: [xs_sdk] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[ INFO] [1712162326.464764714]: [xs_sdk]        Found DYNAMIXEL ID:  8, Model: 'XL430-W250', Joint Name: 'wrist_rotate'.
[ INFO] [1712162326.510764251]: [xs_sdk]        Found DYNAMIXEL ID:  7, Model: 'XM430-W350', Joint Name: 'wrist_angle'.
[ INFO] [1712162326.556774781]: [xs_sdk]        Found DYNAMIXEL ID:  5, Model: 'XM430-W350', Joint Name: 'elbow_shadow'.
[ INFO] [1712162326.602760546]: [xs_sdk]        Found DYNAMIXEL ID:  4, Model: 'XM430-W350', Joint Name: 'elbow'.
[ INFO] [1712162326.648799049]: [xs_sdk]        Found DYNAMIXEL ID:  6, Model: 'XM430-W350', Joint Name: 'forearm_roll'.
[ INFO] [1712162326.694784084]: [xs_sdk]        Found DYNAMIXEL ID:  2, Model: 'XM430-W350', Joint Name: 'shoulder'.
[ INFO] [1712162326.740774828]: [xs_sdk]        Found DYNAMIXEL ID:  9, Model: 'XL430-W250', Joint Name: 'gripper'.
[ INFO] [1712162326.786773084]: [xs_sdk]        Found DYNAMIXEL ID:  3, Model: 'XM430-W350', Joint Name: 'shoulder_shadow'.
[ INFO] [1712162326.832751993]: [xs_sdk]        Found DYNAMIXEL ID:  1, Model: 'XM430-W350', Joint Name: 'waist'.
[ INFO] [1712162326.844152073]: [xs_sdk] Writing startup register values to EEPROM. This only needs to be done once on a robot. Set the `~load_configs` parameter to false from now on.
[ INFO] [1712162327.979795223]: [xs_sdk] The operating mode for the 'arm' group was changed to position.
[ INFO] [1712162328.013759970]: [xs_sdk] The operating mode for the 'gripper' joint was changed to pwm.
[ INFO] [1712162328.226745478]: [xs_sdk] Interbotix 'xs_sdk' node is up!
~~~

It worked well as expected, 

1. The 2 arms' starting positions were both at home/sleep, referring to the following picture.
  ![Both the leader arm wx250s and the follower arm vx300s started with home/sleep positions as expectedly](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/image/ros1_noetic_starting_position.png)

2. When we manually manipulate the master arm wx250s, the puppet arm vx300s imitated the movement of its master correctly, referring to [this video clip](https://youtu.be/mjj88K31q7g). 

  [![The follower arm vx300s didn't exactly imitate the movement of its leader wx250s](https://img.youtube.com/vi/mjj88K31q7g/hqdefault.jpg)](https://www.youtube.com/watch?v=mjj88K31q7g)
 
3. All the source codes, including the configuration files, are stored in [our github repo](https://github.com/housework-robot/main/tree/main/S01_anatomy_of_stanford_aloha/bug_report/ros1_noetic). 
