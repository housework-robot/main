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

## 2.1 ESP32

## 2.2 I2CDev

## 2.3 MPU6050_tockn

## 2.4 SimpleFOC
