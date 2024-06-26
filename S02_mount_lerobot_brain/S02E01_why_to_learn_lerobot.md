# 1. Why to learn LeRobot

The American tech media InfoQ reported on May 16, 2024, that Huggingface, which focuses on large-scale artificial intelligence models, announced the launch of a new project called LeRobot.

LeRobot is dedicated to providing motion planning models, training data, and related tools for real-world robots, aiming to lower the technical barrier of robotics by sharing training data and pre-trained models, thereby promoting the popularization and advancement of robotics technology.

The head of LeRobot, Remi Cadene, formerly a senior scientist at Tesla, said on Twitter, "LeRobot propels the technology of robotics, whose value is akin to Huggingface's transformers toolkit, which has greatly advanced the field of natural language processing." Let's elaborate on Remi Cadene's statement, which may contain three layers of meaning:

## 1.1 Standardized robotic APIs 

Huggingface has developed standardized APIs and toolkits such as transformers and diffusers. The transformers have greatly facilitated the popularization and advancement of natural language processing technology, while diffusers have significantly promoted the spread and progress of image and video generation technology, gaining widespread acceptance in the industry and becoming de facto standards.

LeRobot, following the example of transformers and diffusers, provides standardized APIs and toolkits for robotics technology.

The LeRobot team is using standardized APIs and toolkits to reprogram and develop the currently popular pre-trained motion planning models for robots, thereby promoting the use of standardized APIs and toolkits for programming and developing future robotic motion planning models.

If the robotic motion planning models are programmed and developed using standardized APIs and toolkits, then other teams will find it easier to optimize and iterate on the existing basis. Moreover, assembling multiple models together to form complex workflows will also become easier.


## 1.2 Market for training datasets and robotic models

The standardization of the relevant APIs and toolkits makes it easier for peers in the industry to use pre-trained large models for robotic motion planning.

With the standardized APIs and toolkits, it facilitates the formation of a unified market, making the sale and purchase of pre-trained robotic models easier, and thus promoting the prosperity of the robotics industry.

Not only does this promote the prosperity of the market for pre-trained robotic models, but it also stimulates the prosperity of the market for robot training datasets.


## 1.3 Tutorials to educate robotics engineers and students

Huggingface organizes experts to continuously write a large number of high-quality technical tutorials for AI large models, which has greatly promoted the popularization of AI large model technology.

It is hoped that LeRobot can also replicate this success by organizing experts to write a large number of technical tutorials for robotics.


# 2. Robotic brain, brainstem, and body

To facilitate the standardization of APIs, it is advisable to adopt a divide-and-conquer approach, dividing the robot into three parts: the brain, brainstem, and body.

## 2.1 Robot brainstem

LeRobot utilizes [Gymnasium](https://gymnasium.farama.org/content/basic_usage/) as the hub connecting the brain and the body of the robot, transmitting the action commands from the brain to the body, and then relaying the operational results and environmental information collected by the body, such as videos captured by the body's camera, back to the brain.

Gym was developed by the OpenAI team and later received support from the Deepmind team. Gym uses simulation to train reinforcement learning models and has been widely accepted in the industry, becoming the de-facto standard.

LeRobot applies Gym as the hub connecting the robot's brain and body,

This may raise many questions, such as whether the pre-trained reinforcement learning models can still be functional, when a robot serving as a warrior, is injured and loses some of its functions on the battlefield?

Or, whether generative models like Diffusion Transformer can also be used as the hub connecting the robot's brain and body?

We set aside these questions for the time being, and use Gym to solve some problems, if not all of them.


## 2.2 Robot brain

Currently, LeRobot has reprogramed three robot brains, or more accurately, three robot motion planning models, using their proposed standardized APIs. These models are [aloha](https://link.zhihu.com/?target=https%3A//github.com/tonyzhaozh/aloha/), [diffusion](https://link.zhihu.com/?target=https%3A//github.com/real-stanford/diffusion_policy), and [tdmpc](https://link.zhihu.com/?target=https%3A//github.com/fyhMer/fowm).

All three models are visual-motor models, which generate motion plans output from vision input. It looks likely that LeRobot will use their standardized APIs to reprogram and develop more robotic models.

It is worth to watch closely, in addition to visual-motor models, LeRobot may also include more complex models, such as visual-lingual-motor models that integrate vision, language instructions, and robot actions.

An even more exciting question is whether the robot brain  will evolve from visual-motor model, to visual-lingual-motor model, and then to the multi-modal large language model (MLLM).


## 2.3 Robot body

The current industry consensus is to use ROS (Robot Operating System) to control the hardware of the robot body.

1. [Dora-rs](https://github.com/dora-rs/dora-lerobot) uses shared memory for data transfer, which has improved the performance of data transmission by 17 times compared to ROS2.

2. [Interbotix](https://link.zhihu.com/?target=https%3A//github.com/interbotix) robotic arm organizes their software system into a layered manner, similar to the structure of  operating systems.

    The bottom layer contains various native drivers, and is wrapped into ROS topics and services.

    The middle layer is the kernel, which schedules and manages the lower-level ROS topics and services, optimizing the allocation of computing power and network resources.

    The upper layer is the user space, where ordinary users only need to call Python APIs without worrying about the underlying ROS, greatly reducing the difficulty of use and improving system security.

3. [Robo-gym](https://link.zhihu.com/?target=https%3A//github.com/jr-robotics/robo-gym/blob/master/docs/the_framework.md) also organizes various robotic ROS topics and services into a layered manner, similar to the structure of OS, hence, limiting the inter-operation between the robot brain and the robot body, only through the brainstem which is implemented with gym.

For robots, or more specifically, for humanoid robots, can we develop a humanoid robot operating system based on the practices of Dora-rs, Interbotix, and Robo-gym?


## 2.4 Synergy of physical body and online simulation

By integrating the physical robot body with online simulation, not only it facilitates a low-cost and safer training of robots through simulation, but also improves the user experience when the remote controlling of robots.

One way to achieve the integration of the physical body and online simulation is to start with the 'render' function of gym, and integrate it with game engines such as [Unity](https://unity.com/) and [Unreal Engine](https://www.unrealengine.com/).


# 3. Learning LeRobot in practice

LeRobot covers a wide range of technologies that are rapidly evolving. To quickly get started and apply what you learn, it is better to break down the big goal into smaller tasks, dive straight into problem-solving, and gradually fill in the gaps in your knowledge base during the practical process, rather than first solidifying a comprehensive foundation of the related knowledge, after then starting to learn LeRobot.

1. Use the aloha simulation training data as input, input it to the [ACT](https://github.com/tonyzhaozh/act) robot motion planning model which was reprogrammed by the LeRobot team, and then generate output, which is the robot's motion commands.

2. With the motion commands output generated by the LeRobot model, use it as the input to Interbotix robotic arms, to manipulate the arms to perform various motions.

3. With the video captured by the camera, use it as the input to the ACT motion model.

4. Develop a custom gym environment to connect the LeRobot brain to the robot body, which consists of two Interbotix arms.

5. Collect training dataset, train the ACT model, and then verify its accuracy on the Interbotix arms.
