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
