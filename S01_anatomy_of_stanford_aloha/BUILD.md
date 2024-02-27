## build steps

```
cd ~
## clone repo
git clone --recursive https://github.com/housework-robot/main -b shaobo housework_robot_ws
cd housework_robot_ws/S01_anatomy_of_stanford_aloha

## install ros dependencies
rosdep install --from-paths src --ignore-src -y -r
## build using `symlink-install` options
colcon build --symlink-install

## source workspace and run application
source install/setup.bash
ros2 launch [package_name] [launch_file]
```