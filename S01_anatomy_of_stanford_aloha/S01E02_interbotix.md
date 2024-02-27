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
    * [rviz/xsarm_dual.rviz](./src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_andy/rviz/xsarm_dual.rviz) etc, 

    These modified source codes are stored in this repository. 
