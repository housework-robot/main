# Interbotix physical arm
SO1E03, 2024.03.16

# 1. Objective


[Stanford mobile aloha housework robot](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#hardware-installation) uses 2 pairs of robotic arms.
In last episode, we worked with the [rviz2](https://github.com/ros2/rviz) simulation, and this time, we used the physical interbotix robotic arms, which were shipped to our lab a few days ago. 

First we used one interbotix arm, to execute a series of actions. Then we used two arms to move simultaneously. The purpose was to learn the usage of interbotix robotic arms, but not yet the action learning algorithms. Therefore, in both scenarios, the series of actions were hard-coded in our python programs. 


# 2. Preparation

Step 1. We took an Interbotix vx300 robotic arm out of the box, and connected it to a computer USB port. We used a linux desktop with GPU, [Lamdba Tensorbook](https://lambdalabs.com/deep-learning/laptops/tensorbook/specs).
![Interbotix robotic arm, connected to a Lambda Tensorbook]]"./image/physical_arm_20240315_1.jpeg">

Step 2. Following the suggestion of [Stanford aloha project](https://github.com/MarkFzp/mobile-aloha?tab=readme-ov-file#hardware-installation), downloaded and installed [dynamixel wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).


# 3. Single arm manipulation

Step 1. Detect the USB port

Open the dynamixel wizard, go into "options" page, and configure the options as following. 

   ~~~
   Protocal 2.0
   All ports
   1000000 bps
   ID range from 0-10
   ~~~
<img src="./image/physical_arm_20240315_2.jpeg">


After then, hit "Scan" button, the dynamixel wizard detects that the Interbotix vx300 robotic arm was connected to a USB port, e.g. USB0 port. 
<img src="./image/physical_arm_20240315_3.jpeg">


Step 2. Change the configurations

Double-check whether the modes.yaml and vx300.yaml are configured with the right port, e.g. ttyUSB0. You can refer to [the src of this repo](https://github.com/housework-robot/main/tree/main/S01_anatomy_of_stanford_aloha/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config) for the functional settings. 

~~~
$ gedit /home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/vx300.yaml

"""
# Set the port to USB0, by adding the following content to the first line. 
port: /dev/ttyUSB0
"""


$ gedit /home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/modes.yaml

"""
# Remove the port setting, which usually appeared in the first line of the file.
"""
~~~


Step 3.  Launch the xsarm_control node

Open a terminal, execute the following commands,

~~~
$ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300
[INFO] [launch]: All log files can be found below /home/robot/.ros/log/2024-03-16-21-40-40-156452-robot-test-20901
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [xs_sdk-1]: process started with pid [20922]
[INFO] [robot_state_publisher-2]: process started with pid [20924]
[INFO] [rviz2-3]: process started with pid [20926]
[xs_sdk-1] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-1] [INFO] Using logging level 'INFO'.
[xs_sdk-1] [INFO] Loaded mode configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/modes.yaml'.
[xs_sdk-1] [INFO] Loaded motor configs from '/home/robot/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/vx300.yaml'.
[xs_sdk-1] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  7, Model: 'XM430-W350', Joint Name: 'wrist_rotate'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  6, Model: 'XM430-W350', Joint Name: 'wrist_angle'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  5, Model: 'XM430-W350', Joint Name: 'elbow_shadow'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  4, Model: 'XM430-W350', Joint Name: 'elbow'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  2, Model: 'XM430-W350', Joint Name: 'shoulder'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  8, Model: 'XL430-W250', Joint Name: 'gripper'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  3, Model: 'XM430-W350', Joint Name: 'shoulder_shadow'.
[xs_sdk-1] [INFO]         Found DYNAMIXEL ID:  1, Model: 'XM430-W350', Joint Name: 'waist'.
[xs_sdk-1] [WARN] Writing startup register values to EEPROM. This only needs to be done once on a robot if using a default motor config file, or after a motor config file has been modified. Can set `write_eeprom_on_startup` to false from now on.
[xs_sdk-1] [INFO] The operating mode for the 'arm' group was changed to 'position' with profile type 'time'.
[xs_sdk-1] [INFO] The operating mode for the 'gripper' joint was changed to 'pwm' with profile type 'velocity'.
[xs_sdk-1] [INFO] Interbotix X-Series Driver is up!
[xs_sdk-1] [INFO] [1710596442.089388846] [interbotix_xs_sdk.xs_sdk]: InterbotixRobotXS is up!
[xs_sdk-1] [WARN] [1710596478.708552669] [interbotix_xs_sdk.xs_sdk]: Trajectory rejected since joints are still moving.
[xs_sdk-1] [WARN] [1710596483.193403905] [interbotix_xs_sdk.xs_sdk]: Trajectory rejected since joints are still moving.
[xs_sdk-1] [WARN] [1710596490.214750053] [interbotix_xs_sdk.xs_sdk]: Trajectory rejected since joints are still moving.
[xs_sdk-1] [WARN] [1710596493.724603018] [interbotix_xs_sdk.xs_sdk]: Trajectory rejected since joints are still moving.
[xs_sdk-1] [INFO] [1710596517.516226022] [rclcpp]: signal_handler(signum=2)
[INFO] [robot_state_publisher-2]: process has finished cleanly [pid 20924]
[INFO] [rviz2-3]: process has finished cleanly [pid 20926]
[INFO] [xs_sdk-1]: process has finished cleanly [pid 20922]
~~~

It should automatically open the rviz simulator.
<img src="./image/physical_arm_20240315_5.jpeg">


Step 4. Run a series of predefined actions

Open another terminal, execute the following command,

~~~
$ cd /home/robot/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/demos/python_ros2_api

$ gedit bartender.py
# Change the robot_model from wx250 to vx300.

$ python3 bartender.py 
[INFO] [1710596469.466258201] [robot_manipulation]: 
        Robot Name: vx300
        Robot Model: vx300
[INFO] [1710596469.466468220] [robot_manipulation]: Initialized InterbotixRobotXSCore!
>> andy.nyu: group_info, 
  [group_info]: interbotix_xs_msgs.srv.RobotInfo_Response(mode='position', profile_type='time', joint_names=['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate'], joint_ids=[1, 2, 4, 6, 7], joint_lower_limits=[-3.141582727432251, -1.884955644607544, -2.1467549800872803, -1.7453292608261108, -3.141582727432251], joint_upper_limits=[3.141582727432251, 1.9896754026412964, 1.6057028770446777, 2.1467549800872803, 3.141582727432251], joint_velocity_limits=[3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732], joint_sleep_positions=[0.0, -1.7999999523162842, 1.5499999523162842, 0.800000011920929, 0.0], joint_state_indices=[0, 1, 2, 3, 4], num_joints=5, name=['arm']) 

[INFO] [1710596469.470199764] [robot_manipulation]: 
        Arm Group Name: arm
        Moving Time: 2.00 seconds
        Acceleration Time: 0.30 seconds
        Drive Mode: Time-Based-Profile
[INFO] [1710596469.470370802] [robot_manipulation]: Initialized InterbotixArmXSInterface!
[INFO] [1710596469.975860357] [robot_manipulation]: 
        Gripper Name: gripper
        Gripper Pressure: 50.0%
[INFO] [1710596469.976468017] [robot_manipulation]: Initialized InterbotixGripperXSInterface!
~~~


Now the Interbotix physical vx300 arm should start moving. It is recommended to have someone hold the robotic arm with their hands, and be careful not to get injured by the robotic arm.
<img src="./image/physical_arm_20240315_7.jpeg">



# 4. Dual arms manipulation

Now we challenge ourselves with two robotic arms moving simultaneously. 

Step 1. Detect the USB port

Following the same instruction of " 3. Single arm manipulation -> Step 1 "， we found that in our scenario, the ws250 arm used USB0, and the vx300 arm used USB2 to connect the laptop Lambda Tensorbook. 


Step 2. Change the configurations

Double-check whether the modes_1.yaml and modes_2.yaml are configured with the right ports, e.g. ttyUSB0 for the first arm wx250, ttyUSB2 for the second arm vx300. You can refer to [the src of this repo](https://github.com/housework-robot/main/tree/main/S01_anatomy_of_stanford_aloha/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual/config) for the functional settings. 

~~~
$ gedit /home/robot/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual/config/modes_1.yaml

# Set the port to USB0, by adding the following content to the first line. 
port: /dev/ttyUSB0


$ gedit /home/robot/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual/config/modes_2.yaml

# Set the port to USB0, by adding the following content to the first line. 
port: /dev/ttyUSB2
~~~


Step 3.  Launch the xsarm_dual node

Open the first terminal, execute the following commands,
   
~~~
$ ros2 launch  interbotix_xsarm_dual xsarm_dual.launch.py robot_model_1:=wx250 robot_name_1:=arm_1 robot_model_2:=vx300 robot_name_2:=arm_2 use_dual_rviz:=true
~~~

It should automatically open the rviz simulator.
<img src="./image/simulation_xsarm_dual.jpeg">



Step 4. Run a series of predefined actions

Modify the xsarm_dual.py code, if needed, take a look of [the src of this repo](https://github.com/housework-robot/main/blob/main/S01_anatomy_of_stanford_aloha/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual/demos/xsarm_dual.py) for reference. 


Open another terminal, execute the following command, 

~~~
$ cd ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual/demos/

$ gedit xsarm_dual.py 
"""
...
def main():
    # robot_1 is the "node owner", meaning that it controls the state of rclpy.
    robot_1 = InterbotixManipulatorXS(
        robot_model='wx250',
        robot_name='arm_1',
        moving_time=MOVING_TIME_S,
        node_owner=True,
    )

    # Because robot_1 is the node owner, we set this one's node_owner arg to False.
    robot_2 = InterbotixManipulatorXS(
        robot_model='vx300',
        robot_name='arm_2',
        moving_time=MOVING_TIME_S,
        node_owner=False,
    )
...
"""

$ python3 xsarm_dual.py 
~~~



# 5. Trouble shooting.

If you do not follow the steps above, you might encounter the following error.
<img src="./image/physical_arm_20240315_8.jpeg">

At this point, you can refer to the instructions in "``3. Testing the control of the Interbotix arm``" and follow them, then run the corresponding command.
