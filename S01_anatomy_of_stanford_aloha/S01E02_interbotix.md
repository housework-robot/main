# Manipulate interbotix robotic arm with rviz2
SO1E02, 2024.02.25

# 1. Objective

[Stanford mobile aloha housework robot](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#software-selection----os) uses 2 pairs of robotic arms, which are provided by [Trossen Robotics](https://docs.trossenrobotics.com/interbotix_xsarms_docs/)

It is helpful to learn how to use Interbotix robotic arms, before diving into Stanford aloha robot.

To reduce the learning difficulty, we start to learn interbotix arm using simulatin with [rviz2](https://github.com/ros2/rviz). After then, we use [the physical arms](https://docs.trossenrobotics.com/interbotix_xsarms_docs/).  

The goal of this episode is to use rviz to simulate 2 interbotix arms, one leader arm and one follower arm. We manually predefine a series of actions to control the leader arm, and ask the follower arms to automatically mimick its master. 

[![S01E02 Affordable housework github robot project, interbotix, on youtube](https://img.youtube.com/vi/2XLhbFUmqrI/hqdefault.jpg)](https://www.youtube.com/watch?v=2XLhbFUmqrI)

Click the preview image to see the video on youtube. 


# 2. Install ROS2/Humble and Interbotix

Follow [the instruction of S01E01](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/S01E01_migration.md), to install ROS2/Humble and Interbotix toolkit. 

Notice that interbotix_xsarm_descriptions and interbotix_xsarm_dual are in the list of packages. 

~~~
$ sudo apt install curl
$ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
$ chmod +x xsarm_amd64_install.sh
$ ./xsarm_amd64_install.sh -d humble -p /home/robot/interbotix_ws

$ source /opt/ros/humble/setup.bash
$ source /home/robot/interbotix_ws/install/setup.bash

$ ros2 pkg list | grep interbotix
interbotix_common_modules
interbotix_common_sim
interbotix_common_toolbox
interbotix_moveit_interface
interbotix_moveit_interface_msgs
interbotix_perception_modules
interbotix_perception_msgs
interbotix_perception_pipelines
interbotix_perception_toolbox
interbotix_ros_xsarms
interbotix_ros_xsarms_examples
interbotix_ros_xseries
interbotix_tf_tools
interbotix_xs_driver
interbotix_xs_modules
interbotix_xs_msgs
interbotix_xs_ros_control
interbotix_xs_rviz
interbotix_xs_sdk
interbotix_xs_toolbox
interbotix_xsarm_control
interbotix_xsarm_descriptions
interbotix_xsarm_dual
interbotix_xsarm_joy
interbotix_xsarm_moveit
interbotix_xsarm_moveit_interface
interbotix_xsarm_perception
interbotix_xsarm_ros_control
interbotix_xsarm_sim
~~~


# 3. Get started with interbotix_xsarm_descriptions

Let us play around with rviz2 and interbotix robotic arm. 

Following [interbotix video tutorial: Getting Started With The X-Series Arm](https://www.youtube.com/watch?v=5tH0fmUuCuE&list=PL8X3t2QTE54sMTCF59t0pTFXgAmdf0Y9t&index=5)'s instruction at 1:21, the rviz window will pop up with an interbotix arm. 

~~~
$ ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py robot_model:=vx300s use_joint_pub_gui:=true
[INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-02-25-09-28-20-662472-robot-test-1234903
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [1234924]
[INFO] [joint_state_publisher_gui-2]: process started with pid [1234926]
[INFO] [rviz2-3]: process started with pid [1234928]
~~~

Click the picture, jump to the video download page, download the mp4 file, and view it in browser like chrome. 
[![Control xsarm with interbotix_xsarm_descriptions](./image/xsarm_description.jpeg)](./video/interbotix_description.mp4)


# 4. Study dual arms package interbotix_xsarm_dual


Following [interbotix video tutorial: Working with multiple arms](https://www.youtube.com/watch?v=DnjbNXxBE_8&list=PL8X3t2QTE54sMTCF59t0pTFXgAmdf0Y9t&index=10)'s instruction at at 6:48, we ran interbotix_xsarm_dual package to let 2 arms working together. 

Open two terminals, in the first one, launch the interbotix_xsarm_dual package. 
~~~
$ source /opt/ros/humble/setup.bash
$ source /home/robot/interbotix_ws/install/setup.bash

$ cd /home/robot/interbotix_ws/
$ rosdep install --from-paths src --ignore-src -y    # This step is not mandatory.

$ ros2 launch interbotix_xsarm_dual xsarm_dual.launch.py use_dual_rviz:=true  use_sim:=true
[INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-02-27-22-36-31-570039-robot-test-1455379
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [xs_sdk_sim.py-1]: process started with pid [1455401]
[INFO] [robot_state_publisher-2]: process started with pid [1455403]
[INFO] [xs_sdk_sim.py-3]: process started with pid [1455405]
[INFO] [robot_state_publisher-4]: process started with pid [1455407]
[INFO] [static_transform_publisher-5]: process started with pid [1455409]
[INFO] [static_transform_publisher-6]: process started with pid [1455411]
[INFO] [rviz2-7]: process started with pid [1455413]
[static_transform_publisher-5] [WARN] [1709044592.054890976] []: Old-style arguments are deprecated; see --help for new-style arguments
[static_transform_publisher-6] [WARN] [1709044592.056908492] []: Old-style arguments are deprecated; see --help for new-style arguments
[static_transform_publisher-5] [INFO] [1709044592.065522545] [robot_1_transform_broadcaster]: Spinning until stopped - publishing transform
[static_transform_publisher-5] translation: ('0.000000', '-0.250000', '0.000000')
[static_transform_publisher-5] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-5] from '/world' to '/arm_1/base_link'
[static_transform_publisher-6] [INFO] [1709044592.067838465] [robot_2_transform_broadcaster]: Spinning until stopped - publishing transform
[static_transform_publisher-6] translation: ('0.000000', '0.250000', '0.000000')
[static_transform_publisher-6] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-6] from '/world' to '/arm_2/base_link'
[xs_sdk_sim.py-3] Unknown tag "ros2_control" in /robot[@name='wx200']
[xs_sdk_sim.py-1] Unknown tag "ros2_control" in /robot[@name='wx200']
[xs_sdk_sim.py-3] [INFO] [1709044592.362476243] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
[xs_sdk_sim.py-3] [INFO] [1709044592.363442824] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_dual/share/interbotix_xsarm_dual/config/modes_2.yaml`.
[xs_sdk_sim.py-3] [INFO] [1709044592.363646936] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
[xs_sdk_sim.py-3] [INFO] [1709044592.363812213] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
[xs_sdk_sim.py-3] [INFO] [1709044592.370096123] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
[xs_sdk_sim.py-1] [INFO] [1709044592.373569747] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
[xs_sdk_sim.py-1] [INFO] [1709044592.374507981] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_dual/share/interbotix_xsarm_dual/config/modes_1.yaml`.
[xs_sdk_sim.py-1] [INFO] [1709044592.374719185] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
[xs_sdk_sim.py-1] [INFO] [1709044592.374882373] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
[xs_sdk_sim.py-1] [INFO] [1709044592.381391772] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!

~~~

In the secondary terminal, run xsarm_dual.py. 

~~~
$ source /opt/ros/humble/setup.bash
$ source /home/robot/interbotix_ws/install/setup.bash

$ cd /home/robot/interbotix_ws/
$ rosdep install --from-paths src --ignore-src -y    # This step is not mandatory.
$ cd /home/robot/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual/demos/

$ python3 xsarm_dual.py 
[INFO] [1709044652.836164680] [arm_1.robot_manipulation]: 
        Robot Name: arm_1
        Robot Model: wx200
[INFO] [1709044652.836364354] [arm_1.robot_manipulation]: Initialized InterbotixRobotXSCore!
[INFO] [1709044652.838042881] [arm_1.robot_manipulation]: 
        Arm Group Name: arm
        Moving Time: 2.00 seconds
        Acceleration Time: 0.30 seconds
        Drive Mode: Time-Based-Profile
[INFO] [1709044652.838206496] [arm_1.robot_manipulation]: Initialized InterbotixArmXSInterface!
[INFO] [1709044653.340617131] [arm_1.robot_manipulation]: 
        Gripper Name: gripper
        Gripper Pressure: 50.0%
[INFO] [1709044653.341213526] [arm_1.robot_manipulation]: Initialized InterbotixGripperXSInterface!
[INFO] [1709044653.873589022] [arm_2.robot_manipulation]: 
        Robot Name: arm_2
        Robot Model: wx200
[INFO] [1709044653.874243449] [arm_2.robot_manipulation]: Initialized InterbotixRobotXSCore!
[INFO] [1709044653.878449430] [arm_2.robot_manipulation]: 
        Arm Group Name: arm
        Moving Time: 2.00 seconds
        Acceleration Time: 0.30 seconds
        Drive Mode: Time-Based-Profile
[INFO] [1709044653.880321962] [arm_2.robot_manipulation]: Initialized InterbotixArmXSInterface!
[INFO] [1709044654.387733508] [arm_2.robot_manipulation]: 
        Gripper Name: gripper
        Gripper Pressure: 50.0%
[INFO] [1709044654.388837393] [arm_2.robot_manipulation]: Initialized InterbotixGripperXSInterface!
~~~

Click the picture, jump to the video download page, download the mp4 file, and view it in browser like chrome. 
[![Run interbotix_xsarm_dual package](./image/xsarm_dual_20240228.png)](./video/dual_arms_20240227_2031.mp4)


# 5. Update puppet arms package interbotix_xsarm_puppet

Step 1. Update the interbotix_xsarm_puppet package source code. 

[interbotix_xsarm_puppet package](https://github.com/orgs/Interbotix/discussions/47) was updated yesterday, 2024.2.28. 

![Updated interbotix_xsarm_puppet package source code with ROS2/humble branch](./image/interbotix_xsarm_puppet_fa04fa8.jpg)

You need to download [the updated code from github](https://github.com/Interbotix/interbotix_ros_manipulators/tree/humble/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet), and replace their previous codes. 

A easy-to-use tool to download the github repo branch, is this website, https://download-directory.github.io/

Suppose you have downloaded the updated branch from github, and saved it to a file folder, ~/interbotix_xsarm_puppet_fa04fa8/, then you need to replace interbotix previous codes for interbotix_xsarm_puppet package. 

~~~
$ cd ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/
$ mv -r interbotix_xsarm_puppet/ ~/interbotix_xsarm_puppet_20240229
$ cp -rf ~/interbotix_xsarm_puppet_fa04fa8 .  
~~~

Step 2. Re-compile entire interbotix toolkit. 

Notice that, after compilation, interbotix_xsarm_puppet will show up in the package list. 

~~~
$ cd ~/interbotix_ws
$ rm -rf build/ install/ log/

$ rosdep install --from-paths src --ignore-src -y 
$ colcon build --symlink-install
$ source ~/interbotix_ws/install/setup.bash 

$ ros2 pkg list | grep interbotix
interbotix_common_modules
interbotix_common_sim
interbotix_common_toolbox
interbotix_moveit_interface
interbotix_moveit_interface_msgs
interbotix_perception_modules
interbotix_perception_msgs
interbotix_perception_pipelines
interbotix_perception_toolbox
interbotix_ros_xsarms
interbotix_ros_xsarms_examples
interbotix_ros_xseries
interbotix_tf_tools
interbotix_xs_driver
interbotix_xs_modules
interbotix_xs_msgs
interbotix_xs_ros_control
interbotix_xs_rviz
interbotix_xs_sdk
interbotix_xs_toolbox
interbotix_xsarm_control
interbotix_xsarm_descriptions
interbotix_xsarm_dual
interbotix_xsarm_dual_andy
interbotix_xsarm_joy
interbotix_xsarm_moveit
interbotix_xsarm_moveit_interface
interbotix_xsarm_perception
interbotix_xsarm_puppet
interbotix_xsarm_ros_control
interbotix_xsarm_sim
~~~

Step 3. Launch interbotix_xsarm_puppet package with rviz

Referring to interbotix youtube video tutorial, [X-Series Arms | Working With Multiple Arms](https://www.youtube.com/watch?v=DnjbNXxBE_8) at at 11:06, we need interbotix physical arms, to run the demo. 

With only rviz simulation, when launching interbotix_xsarm_puppet with rviz, we can only see a pair of static arms with no movement. 

![A pair of static arms with no movement in rviz](./image/interbotix_xsarm_puppet_static.png)

we will write a python code in the next section, using a series of predefined action to control the master arm's movement, and then the puppet arm will automatically follow the master's movement. 

~~~
$ ros2 launch interbotix_xsarm_puppet xsarm_puppet.launch.py robot_model_leader:=wx200 robot_model_follower:=wx200 use_sim:=true
[INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-02-29-09-58-08-097330-robot-test-1538813
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [xs_sdk_sim.py-1]: process started with pid [1538835]
[INFO] [robot_state_publisher-2]: process started with pid [1538837]
[INFO] [xs_sdk_sim.py-3]: process started with pid [1538839]
[INFO] [robot_state_publisher-4]: process started with pid [1538841]
[INFO] [xsarm_puppet-5]: process started with pid [1538843]
[INFO] [static_transform_publisher-6]: process started with pid [1538845]
[INFO] [static_transform_publisher-7]: process started with pid [1538847]
[INFO] [rviz2-8]: process started with pid [1538849]
[static_transform_publisher-6] [INFO] [1709171888.600876685] [tf_broadcaster_leader]: Spinning until stopped - publishing transform
[static_transform_publisher-6] translation: ('0.000000', '-0.250000', '0.000000')
[static_transform_publisher-6] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-6] from '/world' to 'leader/base_link'
[static_transform_publisher-7] [INFO] [1709171888.601449875] [tf_broadcaster_follower]: Spinning until stopped - publishing transform
[static_transform_publisher-7] translation: ('0.000000', '0.250000', '0.000000')
[static_transform_publisher-7] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-7] from '/world' to 'follower/base_link'
[xs_sdk_sim.py-3] Unknown tag "ros2_control" in /robot[@name='wx200']
[xs_sdk_sim.py-1] Unknown tag "ros2_control" in /robot[@name='wx200']
[xs_sdk_sim.py-3] [INFO] [1709171888.923684358] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/housework_robot_ws/S01_anatomy_of_stanford_aloha/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
[xs_sdk_sim.py-3] [INFO] [1709171888.924794168] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/follower_modes.yaml`.
[xs_sdk_sim.py-3] [INFO] [1709171888.925022496] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
[xs_sdk_sim.py-3] [INFO] [1709171888.925204707] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
[xs_sdk_sim.py-3] [INFO] [1709171888.931886925] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
[xs_sdk_sim.py-1] [INFO] [1709171888.935820884] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/housework_robot_ws/S01_anatomy_of_stanford_aloha/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
[xs_sdk_sim.py-1] [INFO] [1709171888.936509986] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/leader_modes.yaml`.
[xs_sdk_sim.py-1] [INFO] [1709171888.936729484] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
[xs_sdk_sim.py-1] [INFO] [1709171888.936909864] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
[xs_sdk_sim.py-1] [INFO] [1709171888.944919346] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
[xsarm_puppet-5] [INFO] [1709171889.035756066] [xsarm_puppet]: Ready to start puppetting!
~~~

# 6. Move the puppet arms

The last task of this episode, is to hard-code a series of actions in a python program, to control the master arm, and then automatically ask the puppet arm to follow its master's movement. 

Step 1. Download our modified xsarm_puppet codes from this repository, and replace the previous the previous codes.

* [examples/interbotix_xsarm_puppet/config/leader_modes.yaml](./src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/config/leader_modes.yaml)
* [examples/interbotix_xsarm_puppet/demo/bartender_puppet.py](./src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/demo/bartender_puppet.py)

~~~
$ cd ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet
$ mkdir demo

# Download our modified examples/interbotix_xsarm_puppet/ codes, and replace their previous version. 
~~~

Step 2. re-compile interbotix system.

~~~
$ cd ~/interbotix_ws/
$ rm -rf build/ install/ log/

$ rosdep install --from-paths src --ignore-src -y -r
$ colcon build --symlink-install

~~~

Step 3. In one terminal, launch the xsarm_puppet package. 

~~~
$ cd ~/interbotix_ws/
$ source install/setup.bash

$ ros2 launch interbotix_xsarm_puppet xsarm_puppet.launch.py robot_model_leader:=wx200 robot_model_follower:=wx200 use_sim:=true
[INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-02-29-19-58-37-696287-robot-test-1574573
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [xs_sdk_sim.py-1]: process started with pid [1574595]
[INFO] [robot_state_publisher-2]: process started with pid [1574597]
[INFO] [xs_sdk_sim.py-3]: process started with pid [1574599]
[INFO] [robot_state_publisher-4]: process started with pid [1574601]
[INFO] [xsarm_puppet-5]: process started with pid [1574603]
[INFO] [static_transform_publisher-6]: process started with pid [1574605]
[INFO] [static_transform_publisher-7]: process started with pid [1574607]
[INFO] [rviz2-8]: process started with pid [1574609]
[static_transform_publisher-6] [INFO] [1709207918.208434564] [tf_broadcaster_leader]: Spinning until stopped - publishing transform
[static_transform_publisher-6] translation: ('0.000000', '-0.250000', '0.000000')
[static_transform_publisher-6] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-6] from '/world' to 'leader/base_link'
[static_transform_publisher-7] [INFO] [1709207918.208567190] [tf_broadcaster_follower]: Spinning until stopped - publishing transform
[static_transform_publisher-7] translation: ('0.000000', '0.250000', '0.000000')
[static_transform_publisher-7] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-7] from '/world' to 'follower/base_link'
[xs_sdk_sim.py-1] Unknown tag "ros2_control" in /robot[@name='wx200']
[xs_sdk_sim.py-1] [INFO] [1709207918.494853431] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
[xs_sdk_sim.py-1] [INFO] [1709207918.495822140] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/leader_modes.yaml`.
[xs_sdk_sim.py-1] [INFO] [1709207918.496059587] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
[xs_sdk_sim.py-1] [INFO] [1709207918.496267007] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
[xs_sdk_sim.py-1] [INFO] [1709207918.503874853] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
[xs_sdk_sim.py-3] Unknown tag "ros2_control" in /robot[@name='wx200']
[xs_sdk_sim.py-3] [INFO] [1709207918.533332136] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
[xs_sdk_sim.py-3] [INFO] [1709207918.534300806] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_puppet/share/interbotix_xsarm_puppet/config/follower_modes.yaml`.
[xs_sdk_sim.py-3] [INFO] [1709207918.534504929] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
[xs_sdk_sim.py-3] [INFO] [1709207918.534710849] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
[xs_sdk_sim.py-3] [INFO] [1709207918.542376890] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
[xsarm_puppet-5] [INFO] [1709207918.577568880] [xsarm_puppet]: Ready to start puppetting!
~~~

Step 4. In another terminal, run bartender_puppet.py

~~~
$ cp ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/demo/
$ python3 bartender_puppet.py
[INFO] [1709208030.819269788] [leader.robot_manipulation]: 
	Robot Name: leader
	Robot Model: wx200
[INFO] [1709208030.819511307] [leader.robot_manipulation]: Initialized InterbotixRobotXSCore!
>> andy.nyu: group_info, 
  [group_info]: interbotix_xs_msgs.srv.RobotInfo_Response(mode='position', profile_type='time', joint_names=['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate'], joint_ids=[1, 2, 4, 5, 6], joint_lower_limits=[-3.141582727432251, -1.884955644607544, -1.884955644607544, -1.7453292608261108, -3.141582727432251], joint_upper_limits=[3.141582727432251, 1.972222089767456, 1.623156189918518, 2.1467549800872803, 3.141582727432251], joint_velocity_limits=[3.1415927410125732, 1.0, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732], joint_sleep_positions=[0.0, -1.8799999952316284, 1.5, 0.800000011920929, 0.0], joint_state_indices=[0, 1, 2, 3, 4], num_joints=5, name=['arm']) 

[INFO] [1709208030.821396485] [leader.robot_manipulation]: 
	Arm Group Name: arm
	Moving Time: 2.00 seconds
	Acceleration Time: 0.30 seconds
	Drive Mode: Time-Based-Profile
[INFO] [1709208030.821596503] [leader.robot_manipulation]: Initialized InterbotixArmXSInterface!
[INFO] [1709208031.323066659] [leader.robot_manipulation]: 
	Gripper Name: gripper
	Gripper Pressure: 50.0%
[INFO] [1709208031.323309239] [leader.robot_manipulation]: Initialized InterbotixGripperXSInterface!
~~~

Click the picture, jump to the video download page, download the mp4 file, and view the puppet follower arm mimicks its leader master  arm in browser like chrome. 
[![Write a python program to control the actions of the leader arm, then ask the follower arm to mimic its leader](./image/xarm_puppet_2024029.png)](./video/xarm_puppet_2024029.mp4)
