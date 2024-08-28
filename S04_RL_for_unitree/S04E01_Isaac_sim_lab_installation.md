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


# 2. Install Isaac Sim 

## 2.1 Install Omniverse Launcher

Visit [the official website of Nvidia Isaac Sim](https://developer.nvidia.com/isaac/sim), click on `Download Omniverse`, not `Download Container`.

After filling the user registration information and submitting it, you can download the Omniverse Launcher. The file is not large, only 122MB, and it will be downloaded in a short time.

Be prepared that downloading Isaac Sim will take a long time, and if the network is unstable, you may need to download it repeatedly.

![The official website of Nvidia Isaac Sim](S04E01_asset/2_1_isaac_lab_instruction.png "The official website of Nvidia Isaac Sim")

## 2.2 Install Cache and Nucleus 

Follow the instructions on [the Isaac Sim official website](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#workstation-setup) to install [Cache](https://docs.omniverse.nvidia.com/utilities/latest/cache/installation/workstation.html) and [Nucleus](https://docs.omniverse.nvidia.com/nucleus/latest/workstation/installation.html). 

The download procedure is quite lengthy, but overall it goes relatively smoothly.

![The screenshot of Omniverse launcher](S04E01_asset/2_2_omniverse_launcher.PNG "The screenshot of Omniverse launcher")

## 2.3 Install Isaac Sim

The first major mistake we made was installing the wrong version of Isaac Sim. 

Initially, we installed version 2023.1.1 of Isaac Sim as recommended by the Omniverse Launcher. 

After the installation, Isaac Sim itself worked properly, but we encountered many confusing errors when we proceeded to install and use Isaac Lab. 

After several trials and errors, we found in the installation guide on [the official Isaac Lab website](https://isaac-sim.github.io/IsaacLab/source/setup/installation/binaries_installation.html) that version 4.0 of Isaac Sim can guarantee that Isaac Lab will work properly.

![The official Isaac Lab website](S04E01_asset/2_3_1_isaac_lab_binary.png "The official Isaac Lab website")

However, the 4.0.0 version of Isaac Sim is hidden in a corner of the Omniverse Launcher, easy to be ignored. 

You need to first click to expand the `Release` option, and then click to expand `All Release Builds` to find the 4.0.0 version of Isaac Sim. 

Installing Isaac Sim requires downloading a file of about 14GB, and you need to be very patient for the time consuming download process.

1. Search for `Sim` in the Omniverse Launcher's `Exchange` tab,

![Search for Sim in the Omniverse Launcher](S04E01_asset/2_3_2_isaac_sim_install_1.png "Search for Sim in the Omniverse Launcher")

2. Click to expand `Release`, and then click to expand `All release builds`,

![Click to expand 'Release', and then 'All release builds'](S04E01_asset/2_3_2_isaac_sim_install_2.png "Click to expand 'Release', and then 'All release builds'")  

3. Click to select `Isaac Sim 4.0.0`,

![Click to select 'Isaac Sim 4.0.0'](S04E01_asset/2_3_2_isaac_sim_install_3.PNG "Click to select 'Isaac Sim 4.0.0'") 

4. Click to select `Isaac Sim`, rather than `Headless` and `WebRTC`,

![Click to select 'Isaac Sim'](S04E01_asset/2_3_2_isaac_sim_install_4.PNG "Click to select 'Isaac Sim'") 


## 2.4 Install Isaac Sim Compatibility Checker

It seems optional to install `Isaac Sim Compatibility Checker`. Also it seems optional to install either version 4.1.0 or version 4.0.0.

We installed version 4.1.0. After the installation is complete, running Checker will brings up a window, as shown in the figure below.

1. In the Launcher, click to select `Isaac Sim Compatibility Checker`,

![Click to select 'Isaac Sim Compatibility Checker'](S04E01_asset/2_4_1_isaac_checker_install_1.PNG "Click to select 'Isaac Sim Compatibility Checker'") 

2. Click to select `Release 4.1.0`,

![Click to select 'Release 4.1.0'](S04E01_asset/2_4_1_isaac_checker_install_2.PNG "Click to select 'Release 4.1.0'") 

3. The execution result is a pop-up window containing system information.

![The execution result of the Checker](S04E01_asset/2_4_1_isaac_checker_install_3.PNG "The execution result of the Checker") 


