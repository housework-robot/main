# 1. Two approaches of robot brain transplant

The subject of this article is the brain transplant surgery on the Stanford Aloha robot.

Specifically, it replaces [the Action Chunking with Transformers (ACT)](https://github.com/tonyzhaozh/act) motion planning model on the native Stanford Aloha system with another version of the ACT model reimplemented by LeRobot.

After then we ran the native Stanford aloha system in a simulation environment to executive an insertion task, to verify that the LeRobot brain works well with the native Stanford aloha body. 

There are two approaches to the brain transplant surgery:

1. Using the native Stanford Aloha system as the basic framework, replace the Aloha native brain with the LeRobot's brain. This article adopts this approach.
   
2. Using LeRobot's brain as the basic framework, mount the Stanford Aloha system as the robot's body.

The difference between the two lies in the fact that when using the first approach, the brainstem connecting the brain and the body is the brainstem implemented by the Stanford Aloha team; whereas when using the second approach, the brainstem is implemented by the LeRobot team.


The Stanford Aloha team and the LeRobot team, implement their robot's brainstems, based on the same principle, the [gym-env](https://github.com/Farama-Foundation/Gymnasium) framework:

1. The Stanford Aloha team has implemented two envs: one for simulation, known as [sim-env](https://github.com/tonyzhaozh/act/blob/main/sim_env.py), and the other for the hardware body, known as [real-env](https://github.com/MarkFzp/mobile-aloha/blob/main/aloha_scripts/real_env.py).

2. The LeRobot team's version of [sim-env](https://github.com/huggingface/lerobot/blob/main/lerobot/common/envs/factory.py) re-implemented Stanford Aloha's [sim-env](https://github.com/tonyzhaozh/act/blob/main/sim_env.py), with a more modular system structure, more standardized APIs, and easier configuration of the system through parameter files.


Why did we choose the first approach instead of the more standardized second approach?

1. The native brainstem of the first approach often provides more comprehensive and detailed support for the functionality of the robot body.

2. The more standardized second approach, in its early stages, often sacrifices some less important functions of the robot body in order to emphasize standardization.

3. As the second approach continues to evolve, it is likely to settle on a hybrid architecture, where the main framework of the brainstem achieves 100% standardization, and it contains some sockets, where some native brainstem components can plug-and-play.


To achieve excellence his work, one must first sharpen his tools. First, let us spend some time to prepare the surgery, after then we proceed with the brain transplant surgery:

1. Read LeRobot's source code, to understand the creation, usage, and data format conversion of LeRobot's brain. This step is unrelated to the Stanford Aloha native system.

2. Read Stanford aloha's source code, to understand the the creation, usage, and data format conversion of Stanford Aloha's brain. This step is unrelated to LeRobot's brain.

3. Perform the head transplant surgery, using the Stanford Aloha native system as the framework, and replace the native Aloha brain with LeRobot's brain.



# 2. The creation, usage, and data format conversion of LeRobot's brain

## 2.1 CLI command

## 2.2 the creation of hydra_cfg

## 2.3 the creation of policy

## 2.4 the usage of policy 

## 2.5 the usage of env


# 3. The creation, usage and data format conversion of native Aloha's brain

## 3.1 CLI command

## 3.2 the creation of policy_config

## 3.3 the creation of policy

## 3.4 the usage of policy

## 3.5 the creation of env

## 3.6 the usage of env


# 4. Transplant LeRobot brain to Aloha body

## 4.1 setup PYTHONPATH

## 4.2 CLI command

## 4.3 the creation and usage of LeRobot policy

## 4.4 the creation and usage of native Aloha env

## 4.5 the conversion of LeRobot data format to Aloha's

## 4.6 the conversion of Aloha data format to LeRobot's

## 4.7 the process and result of the exection of Aloha body with LeRobot brain


# 5. Appendix 1. Analysis of the usage of LeRobot brain

## 5.1 The input and output of LeRobot

## 5.2 The creation of LeRobot brain

## 5.3 The usage of LeRobot brain


# 6. Appendix 2. Analysis of the usage of native Stanford aloha brain

## 6.1 The input and output of native Aloha

## 6.2 The creation of native Aloha brain

## 6.3 The usage of native Aloha brain




