# Stanford aloha arms
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


# 2. Source code

1. Download all the source codes from [this repo](https://github.com/housework-robot/main/tree/main/S01_anatomy_of_stanford_aloha/src/interbotix_aloha)

2. Copy them into the directory of "~/interbotix_ws/src", so that the entire file and directory tree looks like the following, 
   ~~~
   $ cd ~/interbotix_ws/src
   $ tree interbotix_aloha
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
    ~~~

5. In another terminal, running the following command, to manipulate the 4 arms and display their movements in rviz simulation. 
   ~~~
   $ cd ~/interbotix_ws/src/interbotix_aloha/demo/
   $ python3 bartender_puppet.py
   ~~~


# 4. Hardware configuration


# 5. Trouble shooting
