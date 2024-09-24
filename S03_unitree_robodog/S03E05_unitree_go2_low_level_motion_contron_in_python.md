# Low-level Motion Control of Unitree Go2 Robotic Dog in Python

# 1. Objectives

Unitree Technology's robot dog not only rivals the top-tier robotics company [Boston Dynamics](https://support.bostondynamics.com/s/topic/0TOUS0000002B7R4AU/get-started-with-spot) in functionality, but also offers a price that is affordable to the general public, being just a fraction of the cost of Boston Dynamics' products. 

Additionally, [unitree's technical documentation and APIs](https://support.unitree.com/home/en/developer/Basic_services) are written in a simple and clear manner, making them very easy to get started with.

Unitree's robot dog, the Unitree-Go2, excels in motion control. Beginners can start with [high-level motion control](https://support.unitree.com/home/en/developer/High_motion_control) and then move on to [low-level motion control](https://support.unitree.com/home/en/developer/Basic_motion_control).

The example codes of Unitree's official tutorials, including the example codes for low-level motion control, are written in C++. 

There are two example c++ codes for low-level motion control, `[go2_low_level.cpp](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_low_level.cpp)` and `[go2_stand_example.cpp](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp)`.

Unitree's official tutorials also provide [articles and example codes in Python](https://github.com/unitreerobotics/unitree_sdk2_python), but it seems that there is currently no Python codes corresponding to `go2_low_level.cpp` and `go2_stand_example.cpp`.

We implemented two python codes, `[go2_low_level.py](S03E05_src/go2_low_level.py)` and `[go2_stand_example.py](S03E05_src/go2_stand_example.py)`. The workflows of these Python codes are essentially the same as their C++ versions, and the naming of variables and functions is also largely the same.
