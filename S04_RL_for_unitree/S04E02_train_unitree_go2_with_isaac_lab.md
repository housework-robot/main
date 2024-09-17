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

