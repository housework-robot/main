# Low-level Motion Control of Unitree Go2 Robotic Dog in Python

# 1. Objectives

Unitree Technology's robot dog not only rivals the top-tier robotics company [Boston Dynamics](https://support.bostondynamics.com/s/topic/0TOUS0000002B7R4AU/get-started-with-spot) in functionality, but also offers a price that is affordable to the general public, being just a fraction of the cost of Boston Dynamics' products. 

Additionally, [unitree's technical documentation and APIs](https://support.unitree.com/home/en/developer/Basic_services) are written in a simple and clear manner, making them very easy to get started with.

Unitree's robot dog, the Unitree-Go2, excels in motion control. Beginners can start with [high-level motion control](https://support.unitree.com/home/en/developer/High_motion_control) and then move on to [low-level motion control](https://support.unitree.com/home/en/developer/Basic_motion_control).

The example codes of Unitree's official tutorials, including the example codes for low-level motion control, are written in C++. 

There are two example c++ codes for low-level motion control, [`go2_low_level.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_low_level.cpp) and [`go2_stand_example.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp).

Unitree's official tutorials also provide [articles and example codes in Python](https://github.com/unitreerobotics/unitree_sdk2_python), but it seems that there is currently no Python codes corresponding to `go2_low_level.cpp` and `go2_stand_example.cpp`.

We implemented two python codes, [`go2_low_level.py`](S03E05_src/go2_low_level.py) and [`go2_stand_example.py`](S03E05_src/go2_stand_example.py). The workflows of these Python codes are essentially the same as their C++ versions, and the naming of variables and functions is also largely the same.

&nbsp;
# 2. sport_mode

Before running the Python program for low-level motion control, we need to do some preparatory work:

1. Press the button on battery shortly, and then press it again and hold long, to start the Go2 robot dog. After starting, the Go2 robot dog will stand upright.

2. Find an Ethernet cable, plug one end into the Go2's network port and the other end into the network port of a Ubuntu computer.

&nbsp;
## 2.1 ifconfig

In a CLI terminal, execute `ifconfig` command, to get the name of the ethernet port. In our case, our ethernet port name is `enx207bd51a15b6`. 

~~~
$ ifconfig
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:d3:d3:b2:85  txqueuelen 0  (Ethernet)
        ...

enx207bd51a15b6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.123.117  netmask 255.255.255.0  broadcast 192.168.123.255
        inet6 fe80::1338:72cc:81de:e0fa  prefixlen 64  scopeid 0x20<link>
        ether 20:7b:d5:1a:15:b6  txqueuelen 1000  (Ethernet)
        RX packets 73  bytes 11731 (11.7 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 67  bytes 10190 (10.1 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        ...

wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.120  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::2585:b1c9:fad1:2c02  prefixlen 64  scopeid 0x20<link>
        ether 70:d8:23:b8:39:36  txqueuelen 1000  (Ethernet)
        ...
~~~

&nbsp;
## 2.2 sport_mode

Following the instruction of unitree's official guide "[Quick Start](https://support.unitree.com/home/en/developer/Quick_start)", open the unitree APP in a mobile phone. 

Click "device", then click "service status", then click "sport_mode", to switch off unitree-go2 sport service, referring to the following video for details. 

[![Switch off unitree go2's sport mode](https://img.youtube.com/vi/iAFIqVQT9VM/hqdefault.jpg)](https://www.youtube.com/watch?v=iAFIqVQT9VM)

