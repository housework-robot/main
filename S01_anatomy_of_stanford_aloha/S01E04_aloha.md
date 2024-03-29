# Clone Stanford aloha robotic arms hardware architecture
SO1E04, 2024.03.28

# 1. Objective


[Stanford mobile aloha housework robot](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#hardware-installation) uses 2 pairs of robotic arms.
In the last two episodes, we manipuated interbotix arms, displaying the movement with [rviz2](https://github.com/ros2/rviz) simulation, and also running with the physical interbotix robotic arms. 

Those manipulations were implemented by calling interbotix APIs, and were not directly related with Stanford aloha project. 

In this episode, we will follow Stanford aloha project's instruction, 

1. to organize the 4 interbotix arms into 2 pairs, each pair consists of one master and one puppet. We will use 2 wx250s interbotix arms as the the masters, and use 2 vx300s arms as the puppets. 
   
2. We will write a python program, to hard code a series actions, and use this program to manipute the 2 master wx250s arms. 

3. We expect to see that the 2 puppet arms will imitate their masters' actions, both in rviz simulation and with physical movements. 

Since Stanford aloha project's codes do not support ROS2/Humble with Ubuntu22.04, the challenges are,

1. Create a ROS2 package for Stanford aloha system, including difference package.xml, launch.py, CMakeLists.txt. 

2. Modify Stanford aloha's python codes, because we use upgraded interbotix's robotic arm's APIs. 

3. Display 4 arms' movements with rviz simulation, this means we need to create a rviz configuation file to display the 4 arms. 

4. Configurate the USB ports to connect the computers to the arms.

[![Clone Stanford aloha robotic arms hardware architecture, on youtube](https://img.youtube.com/vi/D53mRnVwRCs/hqdefault.jpg)](https://www.youtube.com/watch?v=D53mRnVwRCs)
 
Click the preview image to see the video on youtube. 


# 2. Source code

1. Download all the source codes from [this repo](https://github.com/housework-robot/main/tree/main/S01_anatomy_of_stanford_aloha/src/interbotix_aloha)

2. Copy them into the directory of "~/interbotix_ws/src", so that the entire file and directory tree looks like the following, 
   ~~~
   $ cd ~/interbotix_ws/src
   $ tree interbotix_aloha
      interbotix_aloha/
      ├── CMakeLists.txt
      ├── commands.txt
      ├── config
      │   ├── master_modes_left.yaml
      │   ├── master_modes_right.yaml
      │   ├── puppet_modes_left.yaml
      │   └── puppet_modes_right.yaml
      ├── demo
      │   └── bartender_puppet.py
      ├── launch
      │   └── xsarm_puppet.launch.py
      ├── package.xml
      ├── rviz
      │   └── stanford_aloha.rviz
      ├── setup.py
      └── src
          ├── xsarm_puppet_left.cpp
          ├── xsarm_puppet_right.cpp
          └── xsarm_puppet_single.cpp
      5 directories, 14 files
   ~~~

3. Compile the source code, 
    ~~~
    $ cd ~/interbotix_ws
    $ rm -rf build/ install/ log/

    $ colcon build --symlink-install
    $ source install/local_setup.bash
    ~~~

4. Launch the package,
    ~~~
    $ ros2 launch interbotix_aloha xsarm_puppet.launch.py use_sim:=true
      [INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-03-28-23-31-48-002003-robot-test-6086
      [INFO] [launch]: Default logging verbosity is set to INFO
      [INFO] [xs_sdk_sim.py-1]: process started with pid [6110]
      [INFO] [robot_state_publisher-2]: process started with pid [6112]
      [INFO] [xs_sdk_sim.py-3]: process started with pid [6114]
      [INFO] [robot_state_publisher-4]: process started with pid [6116]
      [INFO] [xs_sdk_sim.py-5]: process started with pid [6118]
      [INFO] [robot_state_publisher-6]: process started with pid [6120]
      [INFO] [xs_sdk_sim.py-7]: process started with pid [6122]
      [INFO] [robot_state_publisher-8]: process started with pid [6124]
      [INFO] [xsarm_puppet_left-9]: process started with pid [6126]
      [INFO] [xsarm_puppet_right-10]: process started with pid [6128]
      [INFO] [static_transform_publisher-11]: process started with pid [6130]
      [INFO] [static_transform_publisher-12]: process started with pid [6132]
      [INFO] [static_transform_publisher-13]: process started with pid [6134]
      [INFO] [static_transform_publisher-14]: process started with pid [6136]
      [INFO] [rviz2-15]: process started with pid [6138]
      [static_transform_publisher-14] [INFO] [1711639908.875891201] [tf_broadcaster_puppet_right]: Spinning until stopped - publishing transform
      [static_transform_publisher-14] translation: ('0.500000', '-0.500000', '0.000000')
      [static_transform_publisher-14] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
      [static_transform_publisher-14] from '/world' to 'puppet_right/base_link'
      [static_transform_publisher-13] [INFO] [1711639908.875899797] [tf_broadcaster_master_right]: Spinning until stopped - publishing transform
      [static_transform_publisher-13] translation: ('-0.500000', '-0.500000', '0.000000')
      [static_transform_publisher-13] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
      [static_transform_publisher-13] from '/world' to 'master_right/base_link'
      [static_transform_publisher-12] [INFO] [1711639908.876268139] [tf_broadcaster_puppet_left]: Spinning until stopped - publishing transform
      [static_transform_publisher-12] translation: ('0.500000', '0.500000', '0.000000')
      [static_transform_publisher-12] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
      [static_transform_publisher-12] from '/world' to 'puppet_left/base_link'
      [static_transform_publisher-11] [INFO] [1711639908.876832597] [tf_broadcaster_master_left]: Spinning until stopped - publishing transform
      [static_transform_publisher-11] translation: ('-0.500000', '0.500000', '0.000000')
      [static_transform_publisher-11] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
      [static_transform_publisher-11] from '/world' to 'master_left/base_link'
      [xs_sdk_sim.py-7] Unknown tag "ros2_control" in /robot[@name='vx300s']
      [xs_sdk_sim.py-3] Unknown tag "ros2_control" in /robot[@name='vx300s']
      [xs_sdk_sim.py-1] Unknown tag "ros2_control" in /robot[@name='wx250s']
      [xs_sdk_sim.py-5] Unknown tag "ros2_control" in /robot[@name='wx250s']
      [xs_sdk_sim.py-7] [INFO] [1711639909.217459698] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx250s.yaml`.
      [xs_sdk_sim.py-7] [INFO] [1711639909.218816516] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_aloha/share/interbotix_aloha/config/puppet_modes_right.yaml`.
      [xs_sdk_sim.py-7] [INFO] [1711639909.219070486] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
      [xs_sdk_sim.py-7] [INFO] [1711639909.219280513] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
      [xs_sdk_sim.py-3] [INFO] [1711639909.222326886] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx250s.yaml`.
      [xs_sdk_sim.py-3] [INFO] [1711639909.223665325] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_aloha/share/interbotix_aloha/config/puppet_modes_left.yaml`.
      [xs_sdk_sim.py-3] [INFO] [1711639909.223913874] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
      [xs_sdk_sim.py-3] [INFO] [1711639909.224153346] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
      [xs_sdk_sim.py-1] [INFO] [1711639909.227552245] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx250s.yaml`.
      [xs_sdk_sim.py-5] [INFO] [1711639909.228399627] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx250s.yaml`.
      [xs_sdk_sim.py-1] [INFO] [1711639909.229286653] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_aloha/share/interbotix_aloha/config/master_modes_left.yaml`.
      [xs_sdk_sim.py-1] [INFO] [1711639909.229671259] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
      [xs_sdk_sim.py-1] [INFO] [1711639909.230002403] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
      [xs_sdk_sim.py-5] [INFO] [1711639909.230521554] [interbotix_xs_sdk.xs_sdk_sim]: Loaded mode configs from `/home/robot/interbotix_ws/install/interbotix_aloha/share/interbotix_aloha/config/master_modes_right.yaml`.
      [xs_sdk_sim.py-5] [INFO] [1711639909.230817703] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'all' group was changed to 'position'.
      [xs_sdk_sim.py-5] [INFO] [1711639909.231032758] [interbotix_xs_sdk.xs_sdk_sim]: The operating mode for the 'arm' group was changed to 'position'.
      [xs_sdk_sim.py-3] [INFO] [1711639909.235809417] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
      [xs_sdk_sim.py-7] [INFO] [1711639909.235811675] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
      [xs_sdk_sim.py-1] [INFO] [1711639909.239397023] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
      [xs_sdk_sim.py-5] [INFO] [1711639909.241429163] [interbotix_xs_sdk.xs_sdk_sim]: Interbotix 'xs_sdk_sim' node is up!
      [xsarm_puppet_right-10] [INFO] [1711639909.314339970] [xsarm_puppet_right]: Ready to start puppetting!
      [xsarm_puppet_left-9] [INFO] [1711639909.919623392] [xsarm_puppet_left]: Ready to start puppetting!
    ~~~

5. In another terminal, running the following command, to manipulate the 4 arms and display their movements in rviz simulation. 
   ~~~
   $ cd ~/interbotix_ws/src/interbotix_aloha/demo/
   $ python3 bartender_puppet.py
      [INFO] [1711640020.888943711] [master_left.robot_manipulation]: 
      	Robot Name: master_left
      	Robot Model: wx250s
      [INFO] [1711640020.889197899] [master_left.robot_manipulation]: Initialized InterbotixRobotXSCore!
      >> andy.nyu: group_info, 
        [group_info]: interbotix_xs_msgs.srv.RobotInfo_Response(mode='position', profile_type='time', joint_names=['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate'], joint_ids=[1, 2, 4, 6, 7, 8], joint_lower_limits=[-3.141582727432251, -1.884955644607544, -2.1467549800872803, -3.141582727432251, -1.7453292608261108, -3.141582727432251], joint_upper_limits=[3.141582727432251, 1.9896754026412964, 1.6057028770446777, 3.141582727432251, 2.1467549800872803, 3.141582727432251], joint_velocity_limits=[3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732], joint_sleep_positions=[0.0, -1.7999999523162842, 1.5499999523162842, 0.0, 0.800000011920929, 0.0], joint_state_indices=[0, 1, 2, 3, 4, 5], num_joints=6, name=['arm']) 

      [INFO] [1711640020.891103324] [master_left.robot_manipulation]: 
      	Arm Group Name: arm
      	Moving Time: 2.00 seconds
      	Acceleration Time: 0.30 seconds
      	Drive Mode: Time-Based-Profile
      [INFO] [1711640020.891276501] [master_left.robot_manipulation]: Initialized InterbotixArmXSInterface!
      [INFO] [1711640021.392765756] [master_left.robot_manipulation]: 
      	Gripper Name: gripper
      	Gripper Pressure: 50.0%
      [INFO] [1711640021.392966731] [master_left.robot_manipulation]: Initialized InterbotixGripperXSInterface!
      [INFO] [1711640021.409050670] [master_right.robot_manipulation]: 
      	Robot Name: master_right
      	Robot Model: wx250s
      [INFO] [1711640021.409311831] [master_right.robot_manipulation]: Initialized InterbotixRobotXSCore!
      >> andy.nyu: group_info, 
        [group_info]: interbotix_xs_msgs.srv.RobotInfo_Response(mode='position', profile_type='time', joint_names=['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate'], joint_ids=[1, 2, 4, 6, 7, 8], joint_lower_limits=[-3.141582727432251, -1.884955644607544, -2.1467549800872803, -3.141582727432251, -1.7453292608261108, -3.141582727432251], joint_upper_limits=[3.141582727432251, 1.9896754026412964, 1.6057028770446777, 3.141582727432251, 2.1467549800872803, 3.141582727432251], joint_velocity_limits=[3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732], joint_sleep_positions=[0.0, -1.7999999523162842, 1.5499999523162842, 0.0, 0.800000011920929, 0.0], joint_state_indices=[0, 1, 2, 3, 4, 5], num_joints=6, name=['arm']) 
      
      [INFO] [1711640021.412170845] [master_right.robot_manipulation]: 
      	Arm Group Name: arm
      	Moving Time: 2.00 seconds
      	Acceleration Time: 0.30 seconds
      	Drive Mode: Time-Based-Profile
      [INFO] [1711640021.412440399] [master_right.robot_manipulation]: Initialized InterbotixArmXSInterface!
      [INFO] [1711640021.913889777] [master_right.robot_manipulation]: 
      	Gripper Name: gripper
      	Gripper Pressure: 50.0%
      [INFO] [1711640021.914157659] [master_right.robot_manipulation]: Initialized InterbotixGripperXSInterface!
      Exception in thread Thread-3 (bot_thread_func):
      Traceback (most recent call last):
        File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
          self.run()
        File "/usr/lib/python3.10/threading.py", line 953, in run
          self._target(*self._args, **self._kwargs)
        File "/home/robot/interbotix_ws/src/interbotix_aloha/demo/bartender_puppet.py", line 62, in bot_thread_func
          bot.shutdown()  
        File "/home/robot/interbotix_ws/build/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py", line 177, in shutdown
          rclpy.shutdown()
        File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 126, in shutdown
          _shutdown(context=context)
        File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/utilities.py", line 58, in shutdown
          return context.shutdown()
        File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/context.py", line 100, in shutdown
          raise RuntimeError('Context must be initialized before it can be shutdown')
      RuntimeError: Context must be initialized before it can be shutdown
   ~~~


# 4. Hardware configuration


# 5. Trouble shooting

## 5.1 Fastrtps port failure

![Fastrtps port failure](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/image/fastrtps_failure.png)

If you encounter the above error, the solution is simpley reboot your ubuntu OS.

~~~
$ sudo reboot now
~~~

## 5.2 Simulation movement is not smooth

[![The interbotix robotic arms do not move smoothly in rviz simulation](https://img.youtube.com/vi/OdhbOGEiO4Y/hqdefault.jpg)](https://www.youtube.com/watch?v=OdhbOGEiO4Y)

If the four arms do not move smoothly in rviz simulation, a possible solution consists of several steps. 

1. Change the coordinates of the 4 arms, referring to [the launch file](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/interbotix_aloha/launch/xsarm_puppet.launch.py).

2. Change the display window size of the rviz simulator, referring to [the rviz configuation file](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/interbotix_aloha/rviz/stanford_aloha.rviz).

3. Change the demo python code from single thread to multiple threads, referring to [the demo python code](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/interbotix_aloha/demo/bartender_puppet.py).

After those above, the 4 arms in rviz simulator move like this,

[![The interbotix robotic arms move smoothly in rviz simulation](https://img.youtube.com/vi/Gzb734OmZ6s/hqdefault.jpg)](https://www.youtube.com/watch?v=Gzb734OmZ6s)

