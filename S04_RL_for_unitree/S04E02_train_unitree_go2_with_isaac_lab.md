# Train Unitree Go2 Robot Dog with Isaac Sim/Lab

# 1. Objectives

In [the previous article](https://github.com/housework-robot/main/blob/main/S04_RL_for_unitree/S04E01_Isaac_sim_lab_installation.md), we mentioned that, 

> Training a robot using its physical body is feasible, but the training efficiency is low.
> Moreover, if collisions or other incidents occur during the training process that result in robot damage, the cost of training will be high.
> Training robots using simulation requires the creation of a 3D model of the robot's body, which strictly replicates the angles, torques, forces, and other aspects of each joint of the robot.

On the Unitree official website, there is a tutorial titled "[Basic Motion Control](https://support.unitree.com/home/en/developer/Basic_motion_control)", which explains how to use the Isaac Gym simulation platform to train the Unitree robotic dog Go2 to maintain balance during movement using reinforcement learning motion model.

Currently, Isaac Gym has been replaced by Isaac Lab.

On the Isaac Lab official website, there is a tutorial titled "[Migration Guide: From IsaacGymEnvs](https://isaac-sim.github.io/IsaacLab/source/migration/migrating_from_isaacgymenvs.html)", which provides detailed information on how to migrate a system built on Isaac Gym to Isaac Lab, but it is quite challenging to do the migration.

If you do not wish to delve into how to do the migration, but instead want to quickly start the training of robots using the Isaac Lab simulation platform with reinforcement learning algorithms, the most convenient approach is to try out the examples that come with Isaac Lab.

On the Isaac Lab's github page, there are [ready-to-use source codes](https://github.com/isaac-sim/IsaacLab/tree/main/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/locomotion/velocity/config) to train the Unitree robotic dogs A1, Go1, Go2, and the Unitree humanoid robots H1, G1. The codes are designed to train the Unitree robotic dogs to maintain balance when walking on both flat and rough terrain using the PPO reinforcement learning algorithm.

The task at hand is to use [the PPO reinforcement learning algorithm](https://en.wikipedia.org/wiki/Proximal_policy_optimization) with the Isaac Lab simulation platform, to train the Unitree robotic dog Go2 to walk as long as possible, and keep balanced.


# 2. Train Cartpole

On [Isaac Lab's github page](https://isaac-sim.github.io/IsaacLab), there are a series of tutorials, one of them is titled "[Training with an RL Agent](https://isaac-sim.github.io/IsaacLab/source/tutorials/03_envs/run_rl_training.html)", that explains how to use reinforcement learning algorithms to train a [cartpole](https://www.youtube.com/watch?v=JNKvJEzuNsc), and how to use the checkpoint of the motion model after training.

[Reinforcement Learning (RL)]https://en.wikipedia.org/wiki/Reinforcement_learning() is a generic principle, there are quite some specific algorithms belonging to RL, such as DQN, A2C, PPO, etc.

There are several open-source packages that implement those reinforcement learning algorithms.

Isaac Lab has provided wrappers for the following open-source packages: [stable-baselines3 (sb3)](https://github.com/DLR-RM/stable-baselines3), [rsl_rl](https://github.com/leggedrobotics/rsl_rl), [skrl](https://skrl.readthedocs.io/en/latest/), and [rl_games](https://github.com/Denys88/rl_games), which reduces the difficulty of learning for the end users. Among them,

- [sb3](https://github.com/DLR-RM/stable-baselines3) implements a larger number of algorithms,
  
- [rsl_rl](https://github.com/leggedrobotics/rsl_rl)  only implements the PPO algorithm for the time being, with other algorithms in progress, but rsl_rl can run on GPU, thus more efficient than sb3.

“Training with an RL Agent” 教程中，执行训练任务的指令如下，

Following the instruction of "[Training with an RL Agent](https://isaac-sim.github.io/IsaacLab/source/tutorials/03_envs/run_rl_training.html)", we can execute the following commmand to start the training process, 

~~~
$ cd ${HOME}/IsaacLab

$ ./isaaclab.sh -p source/standalone/workflows/sb3/train.py --task Isaac-Cartpole-v0 --num_envs 64 --video
~~~

Please note,

1. The command `./isaaclab.sh -p` can be replaced with `${HOME}/.local/share/ov/pkg/isaac-sim-4.0.0/python.sh`.
   
  `python.sh` is not equivalent to directly executing `python`, it is script that including installation, environmental variables setting, conda virtual environment setting, in addition to executing python.
  
2. `--num_envs 64` refers to training 64 cartpoles simultaneously. Users can replace 64 with another number based on the computing power of their computer.
   
3. `--video` indicates that the training process is recorded as a video.
   
4. `--headless` option should not be used. When installing Isaac Lab in binary mode, [using `--headless` will result in an error](https://github.com/isaac-sim/IsaacLab/issues/878), with the following error message. The bug is yet to fix. 

~~~
No module named 'omni.kit.window.title'
~~~
