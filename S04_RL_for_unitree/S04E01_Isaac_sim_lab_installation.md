# 1. Objectives

The content of this article is about how to correctly install and use Nvidia's Isaac Sim and Isaac Lab, including what mistakes to avoid. 

Since Isaac Sim and Isaac Lab are in a period of rapid iterative upgrading, the products are unstable, and the installation process is prone to errors, especially it is easy to make multiple mistakes at the same time, leading to many very confusing phenomena.

This article records the process of successfully installing Isaac Sim 4.0.0 and Isaac Lab, on an Ubuntu 22.04 computer equipped with an RTX3070Ti GPU. We don't know the version of Isaac Lab, because there is no place to check the version.

Why do we need Isaac Sim and Isaac Lab? Because We will use them in two application scenarios.

## 1.1 Teleoperation

### 1.1.1 Use case

For example, you are the boss of a nursing home in Japan, and you need cleaners to clean the bathroom and tidy up the rooms, but labor in Japan is very expensive. 

If you hire employees from other countries, you need to go through the visa process, which is cumbersome, and may lead to future troubles, including the impact of immigration on the social order.

The solution of teleoperation is that there are several robots in the Japanese nursing home. When the robots are cleaning the bathroom and tidying up the rooms, foreign employees remotely control the robots from overseas, thus avoiding the various troubles of hiring foreign employees physically working in Japan.

During teleoperation, foreign employees need to be able to "see" the workplace of the nursing home via the Internet.

The simplest solution is to transmit the video captured by the robot's camera to the foreign employees in real time.

By remotely controlling the robot to move and look around, the foreign employees can roughly outline a 3D map of the workplace in their minds.

### 1.1.2 Simulation engine

Is there a way to use a computer, rather than a human brain, to process the video and thereby create a 3D map of the workplace?

One solution is to use a simulation engine to draw the 3D map. We create a 3D map of the workplace in advance, including the 3D models of various furniture and equipments. Then, we apply the 2D images captured by the robot's camera onto the surface of the 3D model, similar to wall paper.

When necessary, we also make minor adjustments and modifications to the pre-made 3D models. 

### 1.1.3 Simulation engine vs game engine

Why use a simulation engine, such as Nvidia Isaac Sim, instead of a game engine like Unity and Unreal, to create the 3D map of workplace?

Game engines do not emphasize the movement of characters and objects, such as collisions and fragmentation, strictly in accordance with the laws of physics.

However, in teleoperation application scenarios, we must strictly adhere to the laws of physics to avoid incorrect operations.

Therefore, we need to use a simulation engine and do not recommend using a game engine to draw the 3D map of the workplace and the 3D models of furniture and equipments.


## 1.2 Robot Training

### 1.2.1 Gymnasium-Robotics

Training a robot using its physical body is feasible, but the training efficiency is low. 

Moreover, if collisions or other incidents occur during the training process that result in robot damage, the cost of training will be high.

Training robots using simulation requires the creation of a 3D model of the robot's body, which strictly replicates the angles, torques, forces, and other aspects of each joint of the robot. 

Additionally, training robot movements often involves the use of reinforcement learning algorithms.

> [Gymnasium-Robotics](https://robotics.farama.org/index.html) is a collection of robotics simulation environments for Reinforcement Learning

Gym is recognized as a de facto industry standard for robotic simulation training, it consists of three main aspects:

1. It has a large collection of 3D models of various robots,
   
2, It has a large collection of reinforcement learning algorithms, with standardized APIs, significantly reducing the learning cost,

3. It has implemented a framework of `environments` that allows different robots to interface with a variety of reinforcement learning algorithms, smoothly.

### 1.2.2 Mujoco vs Isaac Lab

The default simulation engine for Gym is [Mujoco](https://mujoco.org/). 

Initially developed by a PhD from MIT and released in 2012, the development of Mujoco was subsequently led by OpenAI. In 2021, Mujoco was acquired by Deepmind, a Google sub-company. 

While Mujoco is easy to use within the Gym framework, its capabilities seem not as powerful as Nvidia's [Isaac Sim](https://developer.nvidia.com/isaac/sim). 

Building upon Isaac Sim, Nvidia has developed [Isaac Lab](https://developer.nvidia.com/isaac/sim#isaac-lab), which makes it easy to use Isaac Sim within the Gym framework.

