# See the World through Unitree Robotic Dog's Eyes

# 1. Objectives

Robots do not need to pursue full automation from the start. Initially, they can be completely non-automated, relying on human remote control. Then, gradually, they can evolve from being completely non-automated to semi-automated, and ultimately to fully automated.

The advantage of this evolution approach is that the technology is simple, the product is practical, it can quickly generate revenue, and it allows for progress during the commercial practice.

The key issue of remote wireless control is to transmit the video, which is captured by the robot's camera in real-time, wirelessly to the cloud server, and then ask the cloud server to process it and forward the results to the human operators.

The subject of this article is to transmit the video, captured by the eyes of the unitree robotic dog, to a remote server wirelessly and in real-time. It consists of the following tasks:

1. Writing the first program, which is deployed and runs on the Orin embedded computer mounted on the back of the unitree robotic dog, to capture the video from the dog's camera in real time and upload it to the remote server in real time.
   
    The Orin embedded computer and the dog's body are two independent devices, and the Orin can be removed from the back of the dog without any effect on the dog's movement. 
  
    The dog's camera is located on the head of the robotic dog, and it is a part of the dog's body. Therefore, the technical challenge of the first program is how to obtain the video data from the dog's body to the Orin.

2. Writing the second program, which is deployed and runs on the remote server. For the convenience of development and testing, we will temporarily use my local computer as a substitute for the remote server.
   
    The program running in the remote server receives the videoes, which are uploaded by the robotic dog and other devices in real time, and displays those videos from multiple sources on the computer screen.

3. Writing the third program, which is deployed and runs on another computer, to capture video and upload it to the remote server in real time.

    The task of the third program is to confirm that the remote server can simultaneously obtain videos uploaded by multiple robotic dogs and other devices.

![Figure 1. The route of Unitree Go2 dog's video transmissio](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E04_src/video_route.png "Figure 1. The route of Unitree Go2 dog's video transmission")


# 2. Orin system environment

As previously mentioned, the Orin embedded computer and the robotic dog's body are two independent devices. Even when a third-party user crashes the system on the Orin, it will not affect the normal operation of the robotic dog's body.

If we want to deploy and run our own Python programs on the Orin embedded computer, the first step is to install [cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds) version 0.10.x, and the unitree sdk for python, [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) in the Orin. 

## 2.1 Act as user "unitree", rather than "root"

~~~
unitree$ whoami
unitree
unitree$ cd ~ & pwd
/home/unitree
~~~

## 2.2 Install cyclonedds

* Install version 0.10.x, do not install version 0.11.x.

    The communication between the dog's body and the Orin embedded computer on the dog's back is implemented with cyclonedds package.

    The cyclonedds used by the dog's body may be of version 0.10.2, so the cyclonedds running in the Orin must use version 0.10.x, too.

    If the two versions are inconsistent, the Orin and the dog's body cannot establish a DDS communication channel.
  
* When using Cmake, add `-DBUILD_DDSPERF=OFF`.

    If you don't add `-DBUILD_DDSPERF=OFF`, a compilation error will occur when cmake compiles `ddsperf_types.c`.

~~~
unitree$ git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
unitree$ cd cyclonedds && mkdir build install 
unitree$ cd /home/unitree/cyclonedds/build
unitree$ cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_DDSPERF=OFF
unitree$ cmake --build . --target install
~~~

## 2.3 Install unitree_sdk2_python

Set the system environment variable `CYCLONEDDS_HOME` as "/home/unitree/cyclonedds/install", rather than "~/cyclonedds/install". 

~~~
unitree$ export CYCLONEDDS_HOME="/home/unitree/cyclonedds/install"

unitree$ cd /home/unitree
unitree$ git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
unitree$ cd /home/unitree/unitree_sdk2_python
unitree$ pip3 install -e .
~~~


# 3. The first program, cyclonedds, VideoClient & ChannelFactory

As previously stated, the communication between the dog's body and the Orin embedded computer mounted on the dog's back is facilitated by cyclonedds.

For the convenience of programming, unitree_sdk2_python has encapsulated cyclonedds into VideoClient and ChannelFactory, among others.

[Our first Python program](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E04_src/imagemq_dog.py) is in this repo, and we placed it in this directory in Orin, "/home/unitree/unitree_sdk2_python/example/front_camera/imagemq_dog.py".

You need to change the "connect_to" in the program to your computer's IP address, either a public network or a local area network IP address is fine. And your computer is a substitute for the remote server. 

To run this program, execute the following commands. 

Don't execute the commmand with `sudo` user mode, and don't put `eth0` etc as the parameters. 

~~~
unitree$ cd /home/unitree/unitree_sdk2_python
unitree$ export CYCLONEDDS_HOME="/home/unitree/cyclonedds/install"
unitree$ python3 ./example/front_camera/imagemq_dog.py
~~~


# 4. Message Queue patterns

## 4.1 Unitree chooses ZeroMQ for its humanoid robot H1

The communication between the dog's body and the Orin embedded computer uses cyclonedds. The reason for choosing DDS as the communication method may be due to [considerations similar to those of ROS2](https://design.ros2.org/articles/ros_on_dds.html), which chose DDS over ZeroMQ, emphasizing system stability rather than communication efficiency.

When implementing the teleoperation of the unitree humanoid robot H1, the video captured by the robot H1's human-eye camera needs to be transmitted to a server within the local area network. Unitree chooses ZeroMQ as the communication method for this task.

[The choice of ZeroMQ over DDS](https://github.com/unitreerobotics/avp_teleoperate/tree/master/teleop/image_server) for the unitree humanoid robot H1 may be because, in the task of video transmission, there is a greater emphasis on communication efficiency rather than system stability.

## 4.2 Dynamic public IP address

ZeroMQ offers multiple communication methods, among which `pub/sub` and `push/pull` are unidirectional. If using these two communication methods, the dog must act as a publisher or pusher, or more precisely, the Orin embedded computer on the dog's back must act as a publisher or pusher.

The issue is that our server locates in the internet, maybe in a cloud data center, rather than in a local area network. If the cloud server is treated as a subscriber or puller, it must know the dog's public IP address in advance when it starts to connect to the dog's socket.

A possible solution could be of two steps. First, perform a handshake using the `request/reply` method, where the dog reports its dynamic public IP address to the cloud server. After that, the second step is to transmit videos from the Orin to the cloud server. 

However, this is not only troublesome for programming, but also because the dog's dynamic public IP address changes from time to time, it leads to  unstable communication between the dog and the cloud server.

## 4.3 Solution 1: ZeroMQ req/rep

ZeroMQ provides multiple communication methods, among which `req/rep` is bidirectional. 

We can set the dog as the requester and the cloud server as the replier. 

Since the public IP address of the cloud server is static, the dog can conveniently initiate requests to the cloud server. However, the issue arises when using `socket.send()` and `socket.recv()` for image transmission, which requires encoding the video into base64 or similar, making the programming cumbersome.


## 4.4 Solution 2: Use imagezmq

[Imagezmq](https://github.com/jeffbass/imagezmq) is an open-source project based on ZeroMQ, which makes the transmission of images and videos more convenient for programming and more stable in operation. Imagezmq provides two communication methods: one is `req/rep`, and the other is `pub/sub`.

Usually we run imagezmq `pub/sub` in a local network, and the IP address of the publisher can be set to a static local IP address. In our scenario, the dog and the cloud server are not in the same local area network, so we cannot use the `pub/sub` communication method of imagezmq.

We use the `req/rep` of [imagezmq](https://github.com/jeffbass/imagezmq) as the communication method, to enable real-time video transmission between the dog and the cloud server.


# 5. The second program, the hub server of imagezmq

[The second program](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E04_src/imagezmq_server.py) is deployed and running on the cloud server. 

During our development and debugging process, we used a black Razer Tensorbook laptop to substitute for the cloud server.

Before running the program, we need to install [OpenCV](https://opencv.org/get-started/), [ZeroMQ](https://zeromq.org/download/), and [ImageMQ](https://github.com/jeffbass/imagezmq) first.

~~~
robot@tensorbook$ pip3 install opencv-python
robot@tensorbook$ pip3 install pyzmq
robot@tensorbook$ pip3 install imagezmq
robot@tensorbook$ python3 imagezmq_server.py
~~~


# 6. The third program, multiple imageMQ clients

[The third program](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E04_src/imagezmq_client.py) can be deployed and run on multiple computers, including the cloud server and other computers. 

During our development and debugging process, we deployed and ran this program not only on a black Razer Tensorbook laptop but also on a silver MacBook M1.

Before running the program, again, we need to install [OpenCV](https://opencv.org/get-started/), [ZeroMQ](https://zeromq.org/download/), and [ImageMQ](https://github.com/jeffbass/imagezmq) first.

~~~
robot@tensorbook$ pip3 install opencv-python
robot@tensorbook$ pip3 install pyzmq
robot@tensorbook$ pip3 install imagezmq
robot@tensorbook$ python3 imagezmq_client.py
~~~

# 7. The demo of the entire video system

    [![See the world through Unitree robotic dog's eyes](https://img.youtube.com/vi/BEfr-7JqaKo/hqdefault.jpg)](https://www.youtube.com/watch?v=BEfr-7JqaKo)




