# Programming 2-wheel Balancing Robot

by Kan Deng, Yujie Wang, Yaoxuan Wang, Nov. 16, 2024

----------------------------------------------------------

# 1. Objectives

[Arduino SimpleFOCBalancer](https://github.com/simplefoc/Arduino-FOC-balancer) is an open source project that builds a two wheel balancing robot 
based on 2 BLDC motors, 1 [MPU6050 IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit), 1 ESP32 module for Bluetooth communication, 
and [SimpleFOC library](https://github.com/simplefoc).

[Balance_Bot_DengFOC](https://github.com/ToanTech/Balance_Bot_DengFOC) simplifies simpleFOC by focusing on a few selected chip modules and boards. 
In addition it provides more detailed tutorials and cheapers electrical and mechanical components. 

However [the source code of Balance_Bot_DengFOC](https://github.com/ToanTech/Balance_Bot_DengFOC/blob/main/DengFOC%20%E5%B9%B3%E8%A1%A1%E8%BD%A6%E7%A8%8B%E5%BA%8F/Wx_BlueToothBalancer/Wx_BlueToothBalancer.ino) is for windows. 
To migrate it from windows to ubuntu, there are a few technical challenges. 

This document records the steps we took to migrate `Balance_Bot_DengFOC` source code from windows to ubuntu. 

&nbsp;
# 2. Arduino libraries

To migrate `Balance_Bot_DengFOC` from windows to ubuntu, we have to manually install the related libraries in Arduino IDE.  

## 2.1 Network proxy

Before installing the libraries, we need to setup the network proxy, to make it convenient to download the libraries especially those resides in Github. 

The challenge is that there are only a few VPNs available for ubuntu, e.g. [lantern.io](https://lantern.io/), and they are not stable and quite slow for downloading. 

Our solution is to use two computer, one is Ubuntu, the other is either Macbook or Windows. 

1. Find out the IP address and proxy port of the Macbook.

   In the CLI terminal, use `ifconfig` to find the IP address of the macbook. In our case, it is `192.168.0.118`

   ~~~
   $ ifconfig
     wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
           inet 192.168.0.118  netmask 255.255.255.0  broadcast 192.168.0.255
           ...
   ~~~

   Open macbook's `System Preference`, navigate to `Network`, click `Advanced...` at the lower right corner, then select `proxy` tab, here you can find the proxy port.
   In our case, it is `7897`, shown in the following screen snapshot.

   <p align="center">
     <img alt="Macbook proxy port" src="./S06E01_src/macbook_proxy.png" width="50%">
   </p>


2. Configure the proxy of the Ubuntu.

   Open ubuntu's `System Settings`, navigate to `Network`, and configure it using Macbook's IP address and proxy port, shown in the following screen snapshots.

   <p align="center">
     <img alt="Ubuntu network setting" src="./S06E01_src/ubuntu_proxy01.png" width="48%">
     &nbsp;  
     <img alt="Ubuntu proxy" src="./S06E01_src/ubuntu_proxy02.png" width="48%">
   </p>


&nbsp;
## 2.2 ESP32

## 2.3 I2CDev

## 2.4 MPU6050_tockn

## 2.5 SimpleFOC
