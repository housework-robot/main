# Manipulate interbotix robotic arm with rviz2
SO1E02, 2024.02.25

# 1. Objective

[Stanford mobile aloha housework robot](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#software-selection----os) uses 2 pairs of robotic arms, which are provided by [Trossen Robotics](https://docs.trossenrobotics.com/interbotix_xsarms_docs/)

It is helpful to learn how to use Interbotix robotic arms, before diving into Stanford aloha robot.

To reduce the learning difficulty, we start to learn interbotix arm using simulatin with [rviz2](https://github.com/ros2/rviz). After then, we use [the physical arms](https://docs.trossenrobotics.com/interbotix_xsarms_docs/).  

The goal of this episode is to use rviz to simulate 2 pairs interbotix arms, each pair consists of one master arm and one puppet arm. We manually predefine a series of actions to control the 2 master arms, and ask the 2 puppet arms to automatically follow their masters. 


# 2. Installation

Follow [the instruction of S01E01](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/S01E01_migration.md), to install ROS2/Humble and Interbotix toolkit. 

~~~
$ sudo apt install curl
$ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
$ chmod +x xsarm_amd64_install.sh
$ ./xsarm_amd64_install.sh -d humble -p /home/robot/interbotix_ws

$ source /opt/ros/humble/setup.bash
$ source /home/robot/interbotix_ws/install/setup.bash
$ ros2 pkg list | grep interbotix
~~~


# 3. Get started with interbotlix robotic arm and rviz2

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

![xsarm_description with rviz](./image/xsarm_description.jpeg)


# 4. Try dual arms


Following [interbotix video tutorial: Working with multiple arms](https://www.youtube.com/watch?v=DnjbNXxBE_8&list=PL8X3t2QTE54sMTCF59t0pTFXgAmdf0Y9t&index=10)'s instruction at at 6:48, we executed the following command, but failed. 

~~~
$ ros2 launch interbotix_xsarm_dual xsarm_dual.launch use_dual_rviz:=true
Package 'interbotix_xsarm_dual' not found: "package 'interbotix_xsarm_dual' not found, 
searching: ['/home/robot/apriltag_ws/install/apriltag_ros', 
'/home/robot/interbotix_ws/install/interbotix_ros_xsarms_examples', 
'/home/robot/interbotix_ws/install/interbotix_xsarm_moveit_interface', 
'/home/robot/interbotix_ws/install/interbotix_moveit_interface', 
'/home/robot/interbotix_ws/install/moveit_visual_tools', 
'/home/robot/interbotix_ws/install/interbotix_ros_xsarms', 
...
~~~

It looks like for the new version of interbotix toolkit, the 'interbotix_xsarm_dual' old package doesn't exist any more. 

In addition, it looks like that 'interbotix_ros_xsarms_examples' package might be the replacement of the old 'interbotix_xsarm_dual' package. However, when executing the following commands, they didn't work. 

~~~
$ ros2 launch interbotix_ros_xsarms_examples xsarm_dual.launch use_dual_rviz:=true
file 'xsarm_dual.launch' was not found in the share directory of package 'interbotix_ros_xsarms_examples' which is at '/home/robot/interbotix_ws/install/interbotix_ros_xsarms_examples/share/interbotix_ros_xsarms_examples'


$ ros2 launch interbotix_ros_xsarms_examples xsarm_dual.launch.py use_dual_rviz:=true
file 'xsarm_dual.launch.py' was not found in the share directory of package 'interbotix_ros_xsarms_examples' which is at '/home/robot/interbotix_ws/install/interbotix_ros_xsarms_examples/share/interbotix_ros_xsarms_examples'
~~~

We looked into the directory of '/home/robot/interbotix_ws/install/interbotix_ros_xsarms_examples/share/interbotix_ros_xsarms_examples', noticed that it didn't contain the 'launch' subdirectory. 


# 5. Create interbotix_xsarm_dual package from scratch

Follow these steps to re-create a interbotix_xsarm_dual package from scratch, 

1. Execute 'ros2 pkg create' command, so as to create the file skeleon for a ROS2 pacakge. 
   ~~~
    $ cd /home/robot/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples
    $ ros2 pkg create interbotix_xsarm_dual_andy --build-type ament_python
   ~~~
2. Create and/or modify several codes, including 
    * [package.xml](./src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_andy/package.xml),
    * [setup.py](./src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_andy/setup.py), 
    * [launch/xsarm_dual.xml](./src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_andy/launch/xsarm_dual.launch), 
    * [rviz/xsarm_dual.rviz](./src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_andy/rviz/xsarm_dual.rviz) 

    These modified source codes are stored in this repository.

Trouble shooting. 

1. Rviz file contains mistakes. 
    ~~~
    $ ros2 launch interbotix_xsarm_dual_andy xsarm_dual.launch use_dual_rviz:=true  use_sim:=true
    [INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-02-27-13-31-53-714231-robot-test-1392703
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [xs_sdk_sim.py-1]: process started with pid [1392725]
    [INFO] [robot_state_publisher-2]: process started with pid [1392727]
    [INFO] [xs_sdk_sim.py-3]: process started with pid [1392729]
    [INFO] [robot_state_publisher-4]: process started with pid [1392731]
    [INFO] [static_transform_publisher-5]: process started with pid [1392733]
    [INFO] [static_transform_publisher-6]: process started with pid [1392735]
    [INFO] [rviz2-7]: process started with pid [1392737]
    [static_transform_publisher-5] [WARN] [1709011914.170073229] []: Old-style arguments are deprecated; see --help for new-style arguments
    [static_transform_publisher-6] [WARN] [1709011914.171207862] []: Old-style arguments are deprecated; see --help for new-style arguments
    [static_transform_publisher-6] [INFO] [1709011914.182176143] [robot_2_transform_broadcaster]: Spinning until stopped - publishing transform
    [static_transform_publisher-6] translation: ('0.000000', '0.250000', '0.000000')
    [static_transform_publisher-6] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
    [static_transform_publisher-6] from '/world' to '/arm_2/base_link'
    [static_transform_publisher-5] [INFO] [1709011914.182898467] [robot_1_transform_broadcaster]: Spinning until stopped - publishing transform
    [static_transform_publisher-5] translation: ('0.000000', '-0.250000', '0.000000')
    [static_transform_publisher-5] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
    [static_transform_publisher-5] from '/world' to '/arm_1/base_link'
    [rviz2-7] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
    [rviz2-7] [INFO] [1709011914.400218711] [rviz2]: Stereo is NOT SUPPORTED
    [rviz2-7] [INFO] [1709011914.400341700] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
    [rviz2-7] [INFO] [1709011914.410892299] [rviz2]: Stereo is NOT SUPPORTED
    [xs_sdk_sim.py-3] Unknown tag "ros2_control" in /robot[@name='wx200']
    [xs_sdk_sim.py-3] [INFO] [1709011914.465742582] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
    [xs_sdk_sim.py-3] [INFO] [1709011914.466703036] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_dual_andy/share/interbotix_xsarm_dual_andy/config/modes_2.yaml`.
    [xs_sdk_sim.py-3] [INFO] [1709011914.466905864] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
    [xs_sdk_sim.py-3] [INFO] [1709011914.467063614] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
    [xs_sdk_sim.py-3] [INFO] [1709011914.474459280] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
    [xs_sdk_sim.py-1] Unknown tag "ros2_control" in /robot[@name='wx200']
    [rviz2-7] [ERROR] [1709011914.494841630] [rviz2]: PluginlibFactory: The plugin for class 'rviz_default_plugins/Displays' failed to load. Error: According to the loaded plugin descriptions the class rviz_default_plugins/Displays with base class type rviz_common::Panel does not exist. Declared types are  interbotix_xs_rviz/Interbotix Control Panel rviz_visual_tools/RvizVisualToolsGui
    [rviz2-7] [ERROR] [1709011914.495763991] [rviz2]: PluginlibFactory: The plugin for class 'rviz_default_plugins/Selection' failed to load. Error: According to the loaded plugin descriptions the class rviz_default_plugins/Selection with base class type rviz_common::Panel does not exist. Declared types are  interbotix_xs_rviz/Interbotix Control Panel rviz_visual_tools/RvizVisualToolsGui
    [xs_sdk_sim.py-1] [INFO] [1709011914.495778547] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
    [rviz2-7] [ERROR] [1709011914.496065746] [rviz2]: PluginlibFactory: The plugin for class 'rviz_default_plugins/Tool Properties' failed to load. Error: According to the loaded plugin descriptions the class rviz_default_plugins/Tool Properties with base class type rviz_common::Panel does not exist. Declared types are  interbotix_xs_rviz/Interbotix Control Panel rviz_visual_tools/RvizVisualToolsGui
    [rviz2-7] [ERROR] [1709011914.496345358] [rviz2]: PluginlibFactory: The plugin for class 'rviz_default_plugins/Views' failed to load. Error: According to the loaded plugin descriptions the class rviz_default_plugins/Views with base class type rviz_common::Panel does not exist. Declared types are  interbotix_xs_rviz/Interbotix Control Panel rviz_visual_tools/RvizVisualToolsGui
    [rviz2-7] [ERROR] [1709011914.496595062] [rviz2]: PluginlibFactory: The plugin for class 'rviz_default_plugins/Time' failed to load. Error: According to the loaded plugin descriptions the class rviz_default_plugins/Time with base class type rviz_common::Panel does not exist. Declared types are  interbotix_xs_rviz/Interbotix Control Panel rviz_visual_tools/RvizVisualToolsGui
    [xs_sdk_sim.py-1] [INFO] [1709011914.496621698] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_dual_andy/share/interbotix_xsarm_dual_andy/config/modes_1.yaml`.
    [xs_sdk_sim.py-1] [INFO] [1709011914.496806840] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
    [xs_sdk_sim.py-1] [INFO] [1709011914.496956407] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
    [xs_sdk_sim.py-1] [INFO] [1709011914.504201126] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!

    ~~~
   ![xsarm_dual not displayed correctly with rviz](./image/xsarm_dual_wrong_20240227.jpeg)

