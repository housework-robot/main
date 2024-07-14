# 1. Programming unitree-go2 robodog with python

Starting from this article, we begin to explore the use of Python programming to control the Unitree robot dog, Unitree-Go2.

The subject of this article is to learn how to use the Unitree robot dog Python SDK, [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python), to control the robot dog's motions and to obtain the video captured by the robot dog, building a closed-loop feedback control system.

After completing the closed-loop feedback control, in the next article, we will install the LeRobot brain for the robot dog, which will parse the video, recognize the environment, and then intelligently and automatically complete the preset tasks.


The documentation of Unitree is well-written. After a quick review of these documents, one can get started with hands-on practice, and the process is very smooth.

For the basic operations of the robot dog, it is recommended to read the "[Quick Start](https://support.unitree.com/home/en/developer/Quick_start)" section of the "[Go2 SDK Development Guide](https://support.unitree.com/home/en/developer/SDK%20Concepts)".

Once the robot dog is set up and the SDK is installed, we can proceed to the Python programming development for the robot dog.


Although the overall learning process of [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) is quite easy, and the user experience is quite smooth, there are inevitably still some troubles that one might encounter.

Fortunately, these troubles are easily resolved. This article records the methods to solve these troubles.


# 2. Python SDK

Developing applications for the unitree-go2 robot dog with Python can be done in two ways:

1. Using the ROS2 package, [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2).
   
2. Using the Python SDK, [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python).

In [the documentation of unitree](https://github.com/unitreerobotics/unitree_ros2?tab=readme-ov-file#introduction), there is a section explaining the ROS2 package.

~~~
DDS is alos used in ROS2 as a communication mechanism. Therefore, the underlying layers of Unitree Go2, B2, and H1 robots can be compatible with ROS2. ROS2 msg can be direct used for communication and control of Unitree robot without wrapping the SDK interface.
~~~

This means:

1. The unitree_sdk2_python is a wrapper for unitree_ros2, so unitree_sdk2_python is easier to use.
   
2. The unitree_ros2 package may offer more features than unitree_sdk2_python.
   
3. The unitree_ros2 package can utilize various tools from ROS2, such as rviz.


# 3. Unset http/https proxy

## 3.1 Fault reproduction

When we download the SDK code from GitHub, we might encounter the following issues:

~~~
$ git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
Cloning into 'unitree_sdk2_python'...
fatal: unable to access 'https://github.com/unitreerobotics/unitree_sdk2_python.git/': Failed to connect to 127.0.0.1 port 37237 after 0 ms: Connection refused
~~~

## 3.2 Troubleshooting

According to [a post on web](https://www.jianshu.com/p/358acec10c8f), the root cause of the problem is the presence of http/https proxies in the system, which need to be cleared.

We resolved the aforementioned issues using the following methods:

~~~
$ env | grep -i proxy
no_proxy=localhost,127.0.0.0/8,::1
https_proxy=http://127.0.0.1:37237/
NO_PROXY=localhost,127.0.0.0/8,::1
HTTPS_PROXY=http://127.0.0.1:37237/
HTTP_PROXY=http://127.0.0.1:37237/
http_proxy=http://127.0.0.1:37237/

$ unset https_proxy
$ unset http_proxy
$ unset HTTPS_PROXY
$ unset HTTP_PROXY
$ unset no_proxy
$ unset NO_PROXY
~~~

## 3.3 Verification

~~~
$ env | grep -i proxy
$ 

$ git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
Cloning into 'unitree_sdk2_python'...
remote: Enumerating objects: 243, done.
remote: Counting objects: 100% (37/37), done.
remote: Compressing objects: 100% (25/25), done.
remote: Total 243 (delta 18), reused 12 (delta 12), pack-reused 206
Receiving objects: 100% (243/243), 57.72 KiB | 186.00 KiB/s, done.
Resolving deltas: 100% (100/100), done.
~~~


# 4. Network configuration

Following the "[Quick Start](https://support.unitree.com/home/en/developer/Quick_start)" guide in the "[Go2 SDK Development Guide](https://support.unitree.com/home/en/developer/SDK%20Concepts)", we need to configure the subnet of the computer.

~~~
1. Connect one end of the network cable to the Go2 robot, and the other end to the user's computer. Turn on the USB Ethernet of the computer and configure it. The IP address of the onboard computer of the machine dog is 192.168.123.161, so it is necessary to set the USB Ethernet address of the computer to the same network segment as the machine dog. For example, entering 192.168.123.222 ("222" can be changed to other) in the Address field.
~~~

The "[Quick Start](https://support.unitree.com/home/en/developer/Quick_start)" guide suggests using Ubuntu's system tools to configure the computer network, but in our practice, we found that this method did not work.


## 4.1 Fault reproduction

Take the following steps to reproduce the fault. 

1. Use the Ubuntu system tools to set the computer's network configuration.

![Ubuntu system GUI tool for network configuration](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E01_src/ubuntu_sys_config.png "Ubuntu system GUI tool for network configuration")

2. Use ifconfig to check the computer's IP address.

As expected, the computer's IP address should be 192.168.123.117, but the fact is that the computer's IP address is 169.254.218.64.

~~~
$ ifconfig 
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:24:da:e7:62  txqueuelen 0  (Ethernet)
        ...

enx207bd51a15b6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 169.254.218.64  netmask 255.255.0.0  broadcast 169.254.255.255
        inet6 fe80::1338:72cc:81de:e0fa  prefixlen 64  scopeid 0x20<link>
        ether 20:7b:d5:1a:15:b6  txqueuelen 1000  (Ethernet)
        RX packets 1824  bytes 92140 (92.1 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 390  bytes 68082 (68.0 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        ...

wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.117  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::2585:b1c9:fad1:2c02  prefixlen 64  scopeid 0x20<link>
        ether 70:d8:23:b8:39:36  txqueuelen 1000  (Ethernet)
        ...
~~~


## 4.2 Troubleshooting

The solution is to not use Ubuntu's system tools to set the computer's IP address, but instead to use the CLI (Command Line Interface) to perform the operation.

~~~
$ sudo ifconfig enx207bd51a15b6 192.168.123.117 netmask 255.255.255.0 up
~~~


## 4.3 Verification

1. We first checked the computer's IP address using ifconfig, and confirmed that it has indeed been changed to 192.168.123.117.

~~~
$ ifconfig 
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ...

enx207bd51a15b6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.123.117  netmask 255.255.255.0  broadcast 192.168.123.255
        inet6 fe80::1338:72cc:81de:e0fa  prefixlen 64  scopeid 0x20<link>
        ether 20:7b:d5:1a:15:b6  txqueuelen 1000  (Ethernet)
        RX packets 1951  bytes 98550 (98.5 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 428  bytes 73908 (73.9 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        ...

wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.117  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::2585:b1c9:fad1:2c02  prefixlen 64  scopeid 0x20<link>
        ...
~~~

2. Then, ping the robot dog to confirm that the Ethernet connection is successful.

Note that the robot dog's IP address is fixed, 192.168.123.161.

~~~
$ ping 192.168.123.161
PING 192.168.123.161 (192.168.123.161) 56(84) bytes of data.
64 bytes from 192.168.123.161: icmp_seq=1 ttl=64 time=0.495 ms
64 bytes from 192.168.123.161: icmp_seq=2 ttl=64 time=0.205 ms
...
~~~


# 5. Cyclone DDS installation

[DDS](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html), which stands for Data Distribution Service, is a standard protocol for distributed middleware, with multiple implementations by various vendors, including Eclipse's [Cyclone DDS](https://projects.eclipse.org/projects/iot.cyclonedds).

The unitree-go2 robot dog requires the installation and use of [Cyclone DDS](https://projects.eclipse.org/projects/iot.cyclonedds).


## 5.1 Fault reproduction

1.  Install cyclone DDS python package

~~~
$ pwd
/home/robot/unitree/unitree_sdk2_python

$ sudo pip3 install cyclonedds
Collecting cyclonedds
  Downloading cyclonedds-0.10.5-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl (6.6 MB)
  ...

$ sudo pip3 install -e .
Obtaining file:///home/robot/unitree/unitree_sdk2_python
  Preparing metadata (setup.py) ... done
...
Successfully installed cyclonedds-0.10.2 opencv-python-4.10.0.84 unitree-sdk2py
~~~

2. Runtime error 
   
It is not sufficient to install cyclone DDS python package only, you may encounter a fault that the cyclone module cannot be found, when you run applications.

~~~
$ sudo python3 ./example/helloworld/publisher.py
Traceback (most recent call last):
  File "/home/robot/unitree/unitree_sdk2_python/./example/helloworld/publisher.py", line 3, in <module>
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
  ...
  File "/home/robot/unitree/unitree_sdk2_python/unitree_sdk2py/idl/builtin_interfaces/msg/dds_/_Time_.py", line 13, in <module>
    import cyclonedds.idl as idl
ModuleNotFoundError: No module named 'cyclonedds'
~~~


## 5.2 Troubleshooting

The solution is to install cyclonedds by compiling the source code. The steps to do this are as follows:

~~~
$ pwd
/home/robot/unitree/

$ git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
Cloning into 'cyclonedds'...
remote: Enumerating objects: 46498, done.
remote: Counting objects: 100% (497/497), done.
remote: Compressing objects: 100% (224/224), done.
remote: Total 46498 (delta 265), reused 429 (delta 257), pack-reused 46001
Receiving objects: 100% (46498/46498), 25.33 MiB | 7.12 MiB/s, done.
Resolving deltas: 100% (32097/32097), done.

$ cd cyclonedds/
$ mkdir build
$ ls 
azure-pipelines.yml  cmake             colcon.pkg       CYCLONEDDS_QUALITY_DECLARATION.md  examples  LICENSE                 package.xml      README.md   SECURITY.md
build                CMakeCPack.cmake  compat           docs                               fuzz      NOTICE.md               PkgConfig.pc.in  ROADMAP.md  src
CHANGELOG.rst        CMakeLists.txt    CONTRIBUTING.md  etc                                hooks     PackageConfig.cmake.in  ports            scripts     WiX
$ cd build/

$ cmake .. -DCMAKE_INSTALL_PREFIX=../install
-- The C compiler identification is GNU 11.4.0
-- The CXX compiler identification is GNU 11.4.0
...

$ cmake --build . --target install
[  0%] Building C object src/tools/idlpp/CMakeFiles/idlpp.dir/src/directive.c.o
[  0%] Building C object src/tools/idlpp/CMakeFiles/idlpp.dir/src/eval.c.o
...
~~~


## 5.3 Verification

We use a publish/subscribe (pub/sub) communication sample application, to verify that the installation of cyclonedds is correct and to ensure that DDS is functioning properly.

Open two terminals, and run the publisher and subscriber in one terminal respectively.

1. Publisher

~~~
$ sudo python3 ./example/helloworld/publisher.py
Waitting for subscriber.
...
Publish success. msg: UserData(string_data='Hello world', float_data=1720864195.4757297)
...
~~~

2. Subscriber

~~~
$ sudo python3 ./example/helloworld/subscriber.py
Subscribe success. msg: UserData(string_data='Hello world', float_data=1720864195.4757297)
...
~~~


# 6. Code error

## 6.1 Fault reproduction

When we ran the high_level/read_highstate.py example, we encountered the following error:

~~~
$ sudo python3 ./example/high_level/read_highstate.py enx207bd51a15b6
1720865805.438975 [0]    python3: enp3s0: does not match an available interface.
[ChannelFactory] create domain error. msg: Occurred upon initialisation of a cyclonedds.domain.Domain
Traceback (most recent call last):
  File "/home/robot/unitree/unitree_sdk2_python/./example/high_level/read_highstate.py", line 18, in <module>
    ChannelFactoryInitialize(0, "enp3s0")
  File "/home/robot/unitree/unitree_sdk2_python/unitree_sdk2py/core/channel.py", line 290, in ChannelFactoryInitialize
    raise Exception("channel factory init error.")
Exception: channel factory init error.
~~~


## 6.2 Troubleshooting

You need to make a simple repair to the source code at [~/unitree_sdk2_python /example/high_level/read_highstate.py](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/high_level/read_highstate.py).

~~~
# /home/robot/unitree/unitree_sdk2_python/example/high_level/read_highstate.py
...
if __name__ == "__main__":
    # ChannelFactoryInitialize(0, "enp3s0")
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0, "enx207bd51a15b6")

    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)

    while True:
        time.sleep(10.0)
~~~


## 6.3 Verification

~~~
$ sudo python3 ./example/high_level/read_highstate.py enx207bd51a15b6
Position:  [-0.13875612616539001, 0.5343762040138245, 0.3083520829677582]
Velocity:  [-0.008206428959965706, 0.005749342031776905, -0.015356117859482765]
Yaw velocity:  -0.017044229432940483
Foot position in body frame:  [0.18246731162071228, -0.14312629401683807, -0.3066321909427643, 0.18082867562770844, 0.14395472407341003, -0.30894577503204346, -0.2074877917766571, -0.15374241769313812
, -0.30870190262794495, -0.20652782917022705, 0.15827059745788574, -0.3101361393928528]
Foot velocity in body frame:  [0.0020409778226166964, 0.018983902409672737, -0.014126288704574108, -0.011396055109798908, 0.009530807845294476, 0.01058465801179409, 0.0002106300089508295, 0.0132483020
42484283, -0.002242059912532568, -0.011161339469254017, -0.01471808459609747, 0.0004662753199227154]
Position:  [-0.13875912129878998, 0.5343718528747559, 0.30835217237472534]
Velocity:  [-0.00843083206564188, 0.005642993375658989, -0.015270555391907692]
Yaw velocity:  -0.015978964045643806
...
~~~

