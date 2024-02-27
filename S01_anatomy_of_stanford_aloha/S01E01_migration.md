# Migrate Stanford aloha from ROS1 to ROS2
SO1E01, 2024.02.18

# 1. Objective

So far [Stanford mobile aloha housework robot](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#software-selection----os) is running on Ubuntu 20.04 and ROS1/noetic, and Stanford will migrate their system to Ubuntu 22.04 and ROS2. 

We will help to migrate Stanford aloha to Ubuntu 22.04/jammy and ROS2/humble. 

In the process of migration, it will help us to better understand the source codes of Stanford aloha project. 

[![Anatomy of housework robot video on youtube](https://img.youtube.com/vi/XVGTwpWPCrI/hqdefault.jpg)](https://www.youtube.com/watch?v=XVGTwpWPCrI)

Click the preview image to see the video on youtube. 

# 2. Preparation

1. We use a linux desktop with GPU, [Lamdba Tensorbook](https://lambdalabs.com/deep-learning/laptops/tensorbook/specs).

2. [Ubuntu 22.04.3 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) is installed on this desktop. 

3. [Conda is uninstalled](https://docs.anaconda.com/free/anaconda/install/uninstall/), because it causes many troubles. 

4. Connection routing, 
   
   some people complain they cannot connnect to githubusercontent. A solution is to modify /etc/hosts file, and add githubusercontent's IP address, like 185.199.108.133 to it. 
   ~~~
    # /etc/hosts

    127.0.0.1       localhost
    127.0.1.1       robot-test
    185.199.108.133 raw.githubusercontent.com

    # The following lines are desirable for IPv6 capable hosts
    ::1     ip6-localhost ip6-loopback
    fe00::0 ip6-localnet
    ff00::0 ip6-mcastprefix
    ff02::1 ip6-allnodes
    ff02::2 ip6-allrouters
   ~~~



# 3. Install ROS2/Humble

1. Following [ROS2/Humble official website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), to install ROS2/Humble.


2. Install some useful ROS2 packages,
   ~~~
    $ sudo apt-get install ros-humble-usb-cam
    $ sudo apt-get install ros-humble-cv-bridge
    $ source /opt/ros/humble/setup.sh
   ~~~

3. To verify the success of installation, and to learn ROS2 quickly, following chapters of ROS2 tutorials are helpful,

   ~~~
   $ printenv | grep -i ROS
   ~~~

    * [Configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#configuring-environment), 
  
        If environmental settings are not configured correctly, we may encounter many problems later on. Therefore, it is worthy to spend some time to get familiar with ROS2 environment configurations.  

    * Understanding [nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html), [topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html), [services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html), and [actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html) is essential.

    * [Launching nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html), 
  
        because Stanford mobile aloha robot consists of multiple nodes, launch xml is the enterpoint to start up the Stanford aloha.


# 4. Install Interbotix robot toolkit

1. Following [Trossen interbotix official documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html#amd64-architecture) to install interbotix robot toolkit. 

    * Since we install the interbotix system on Ubuntu 22.04, we should follow the section on AMD64 architecture. 

    * In case you encounter difficulty to connect to https://raw.githubusercontent.com, one soluction refers to the section above, ["2. Preparation, (4) Connection routing"](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/S01E01_migration.md#2-preparation). 

2. As a shortcut, you can take our practice as a reference. 
   ~~~
    $ rosversion -d
    humble
    $ cd /home/robot/

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
    
    $ chmod +x xsarm_amd64_install.sh
    $ ./xsarm_amd64_install.sh -d humble -p /home/robot/interbotix_ws

    $ source /opt/ros/humble/setup.bash
    $ source /home/robot/interbotix_ws/install/setup.bash
    $ ros2 pkg list | grep interbotix
   ~~~

3. To verify the installation is successful, and to enjoy the beauty of interbotix, you can run the rviz to demonstrate the movement of interbotix robot arm. 

    * The detailed tutorial refers to [ROS 2 Quickstart Guide](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/quickstart.html#ros-2-quickstart-guide)

    * As a shortcut, you can watch [Interbotix tutorial video on Rviz simulation](https://www.youtube.com/watch?v=p0hmgNEqU8Q&list=PL8X3t2QTE54sMTCF59t0pTFXgAmdf0Y9t&index=7) on youtube. 

4. Once you have already downloaded the interbotix source code from github, for some reason, you want to reinstall it, you can execute the following commands.
   ~~~
   $ cd /home/robot/interbotix_ws
   $ rm -rf build/ install/ log/
   $ ros2 pkg list | grep interbotix
   $ 
   $ colcon build
   $ source /home/robot/interbotix_ws/install/setup.bash
   ... ...
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
   interbotix_xsarm_joy
   interbotix_xsarm_moveit
   interbotix_xsarm_moveit_interface
   interbotix_xsarm_perception
   interbotix_xsarm_ros_control
   interbotix_xsarm_sim
   $
   $ source /home/robot/interbotix_ws/install/setup.sh
   ~~~


# 5. Migrate Stanford aloha software

1. Download [Stanford aloha source code from github](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#software-installation---ros). 

   As a shortcut, you may take our practice as a reference,

   ~~~
    $ cd /home/robot/
    $ git clone https://github.com/MarkFzp/mobile-aloha.git

    $ cp -rf /home/robot/mobile-aloha /home/robot/interbotix_ws/src/stanford_aloha
   ~~~

2. Following [ROS2 tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) to create 'stanford_aloha' python package. 

    As a shortcut, you can take our practice as a reference,

    ~~~
    $ cd /home/robot/interbotix_ws/src
    $ ros2 pkg create --build-type ament_python stanford_aloha
    ~~~


3. Copy the files in '/home/robot/mobile-aloha/' to '/home/robot/interbotix_ws/src/stanford_aloha/', 

    * Don't change the file structure of '/home/robot/interbotix_ws/src/stanford_aloha/', because the file structure is crucial for ROS packaging.

   * In Stanford aloha's scenario, it is a pure python package, hence, we don't need ament_cmake, CMakeLists.txt, and other c++ related tools and files. 

    The file structure of 'stanford_aloha' python package is like the following, 
   
   ~~~
    $ tree /home/robot/interbotix_ws/src/stanford_aloha/
    /home/robot/interbotix_ws/src/stanford_aloha/

    ├── commands.txt
    ├── config
    │   ├── master_modes_left.yaml
    │   ├── master_modes_right.yaml
    │   ├── puppet_modes_left.yaml
    │   └── puppet_modes_right.yaml
    ├── hardware
    │   ├── aloha_gripper_assembly.pdf
    │   ├── rsd405_belly_cam_mount_v2.stl
    │   ├── rsd405_overhead_cam_mount_v2.stl
    │   ├── rsd405_wrist_mount_v2.stl
    │   ├── Shim_rotor_v1.STL
    │   ├── viperx_gripper_stl.zip
    │   ├── viperx_joint1_adapter.zip
    │   ├── widowx_gripper_std.zip
    │   └── W-shim_rotor_v1.STL
    ├── launch
    │   └── 4arms_teleop.launch
    ├── package.xml
    ├── resource
    │   └── stanford_aloha
    ├── setup.cfg
    ├── setup.py
    ├── stanford_aloha
    │   ├── auto_record.sh
    │   ├── constants.py
    │   ├── dynamixel_client.py
    │   ├── example_waypoint_pid.py
    │   ├── get_episode_len.py
    │   ├── __init__.py
    │   ├── one_side_teleop.py
    │   ├── real_env.py
    │   ├── realsense_test.py
    │   ├── record_episodes.py
    │   ├── replay_and_record_episodes.py
    │   ├── replay_episodes.py
    │   ├── robot_utils.py
    │   ├── sleep.py
    │   ├── speed_test.py
    │   ├── test.ipynb
    │   ├── visualize_episodes.py
    │   └── waypoint_control.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py

   ~~~

4. The source code of the original Stanford aloha project needs to be modify, 

    You can copy and paste the content of the codes in ['src/stanford_aloha
/'](https://github.com/housework-robot/main/tree/main/S01_anatomy_of_stanford_aloha/src/stanford_aloha
) subfolder of this repository to your desktop. 

    There are quite some differences between our code comparing with Stanford's original code. Detailed explanation may come later on. 

5. Build stanford_aloha python package,

    As a shortcut, you may take our practice as a reference, 

    ~~~
    $ cd /home/robot/interbotix_ws/
    $ rm -rf install/ build/ log/

    $ colcon build 
    ... RuntimeWarning:        
    ERROR: setuptools==58.2.0 is used in combination with setuptools_scm>=8.x
    # You can ignore the above warnings and errors

    $ source install/local_setup.bash   
    # Don't forget to source the environmental settings, otherwise, 'stanford_aloha' cannot be shown in the ros2 package list.   
    
    $ ros2 pkg list | grep aloha
    stanford_aloha
    $     
    ~~~

# 6. Hardware configuration

1. Following [Stanford mobile aloha guidance on hardware installation](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#hardware-installation) modify [/etc/udev/rules.d/99-fixed-interbotix-udev.rules](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/misc/99-interbotix-udev.rules). 

    Notice when you install the hardwares including interbotix arms and webcams, you need to change the serial numbers in the "99-interbotix-udev.rules" file. 

2. To active the change to the udev rules with symlinks, execute the following command in a terminal, in any directory. 

    ~~~
    sudo udevadm control --reload && sudo udevadm trigger
    ~~~

3. To verify the success of hardware configuration, you can follow Stanford mobile aloha github repo's [teleoperation testing guidance](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#testing-teleoperation). 
   
   As a shortcut, you can take our practice as a reference. 

   * Open a terminal, execute the following command, and 4 rviz windows will pop up, simulating the 4 interbotix arms. 
   
    ~~~
    $ cd /home/robot/interbotix_ws/src/stanford_aloha/launch
    $ ros2 launch 4arms_teleop.launch     
    ~~~

    Notice that you can safely ignore the following warnings and errors. 
    ~~~
    [usb_cam_node_exe-18] [ERROR] [1708524145.550032124] [camera_calibration_parsers]: Unable to open camera calibration file [/home/robot/.ros/camera_info/default_cam.yaml]
    [usb_cam_node_exe-18] [WARN] [1708524145.550051058] [usb_cam_right_wrist]: Camera calibration file /home/robot/.ros/camera_info/default_cam.yaml not found
    ~~~

   * Open another terminal, 
   
    ~~~
    $ cd /home/robot/interbotix_ws/src/stanford_aloha/stanford_aloha
    $ python3 one_side_teleop.py left
    ~~~
