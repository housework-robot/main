#!/usr/bin/env python3

# Modified from interbotix_ros_manipulators/interbotix_ros_xsarms/examples/python_demos/bartender.py
# by andy.nyu, 2024.02.29 

import sys

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
from rclpy.duration import Duration
import threading 

"""
This script makes the end-effector perform pick, pour, and place tasks.
Note that this script may not work for every arm as it was designed for the wx250.
Make sure to adjust commanded joint positions and poses as necessary.

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250

Then change to this directory and type:

    python3 bartender.py
"""

# python3 bartender_puppet.py

MOVING_TIME_S = 1
SLEEP_DURATION = Duration(seconds=MOVING_TIME_S)

def bot_thread_func(bot):  
    def wait() -> None:  
        """Sleep for SLEEP_DURATION."""  
        bot.core.get_clock().sleep_for(SLEEP_DURATION)  
  
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)  
    wait()  
    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)  
    wait()  
    bot.gripper.release()  
    wait()  
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)  
    wait()  
    bot.gripper.grasp()  
    wait()  

    # bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    # bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    # bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    # bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    
    bot.gripper.release()  
    wait()  
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)  
    wait()  
    bot.arm.go_to_home_pose()  
    wait()  
    bot.arm.go_to_sleep_pose()  
    bot.shutdown()  

def main():
 
    bot = InterbotixManipulatorXS(
        robot_model='wx250s',
        robot_name = 'master_left',
        group_name='arm',
        gripper_name='gripper'
    )

    bot2 = InterbotixManipulatorXS(
        robot_model='wx250s',
        robot_name = 'master_right',
        group_name='arm',
        gripper_name='gripper'
    )

    bot_thread = threading.Thread(target=bot_thread_func, args=(bot,))  
    bot2_thread = threading.Thread(target=bot_thread_func, args=(bot2,))  
  
    bot_thread.start()  
    bot2_thread.start()  
  
    bot_thread.join()  
    bot2_thread.join()  

if __name__ == '__main__':
    main()
