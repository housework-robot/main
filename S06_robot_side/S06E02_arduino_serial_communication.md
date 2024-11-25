# Serial Communication between Arduino and Python

              by Kan Deng, Nov 25, 2024

-------------------

## 1. Objectives

We want to integrate [D-Robotics' RDK-X5 board](https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x5) 
with [DengFOC driver board](https://github.com/ToanTech/Deng-s-foc-controller) for Brushless DC Electric Motors (BLDC) 
in [a simple balancing bot](https://github.com/ToanTech/Balance_Bot_DengFOC). 

![image of the boards]()

In more detail, 

1. The balancing bot has two 2204 brushless motors, two AS5600 motor sensors, one MPU6050 IMU sensor. These motors and sensors are controlled by a DengFOC driver board.

2. [A ESP32 (wemos lolin32 lite) module](https://mischianti.org/esp32-wemos-lolin32-lite-high-resolution-pinout-and-specs/) is connected to the DengFOC driver board. The ESP32 takes charge of running Arduino codes, control the DengFOC driver, and communicating with a RDK-X5 board. 

3. A RDK-X5 board is assembled on the back of the balancing bot, and is connected to a RDK binocular camera on the top of the bot. 

4. RDK-X5 board behaves like an organizer, it receives the video stream captured by the RDK's camera,
   and pushes to the remote [SRS streaming server](https://ossrs.io/lts/en-us/docs/v6/doc/getting-started).

5. RDK-X5 uses AI large model to analyze the video stream, and recognize the environment for navigation.

6. RDK-X5 receives the speeds of the motors, the pose of the balancing bot, from the ESP32 module and DengFOC driver board.
   
7. Taking the environmental analysis result and the motion status of the balancing bot as inputs,
   the RDK-X5 runs a policy model to output the command, which usually consists of speed, steering angle etc. 

In order to empower the RDK-X5 to receive motion status from the ESP32, and send command to them, we need to implement a communication channel between the RDK-X5 board and the ESP32 module. 

Since both the RDK-X5 board and the ESP32 module have wifi and bluetooth, wireless communication is one option. 

Another option is to use cable, especially for serial communication. 

Look into the pinouts of the RDK-X5 board and ESP32 wemos lolin32 lite model, UART, I2C, I2S and SPI are all solution candidates. 

![image of pinouts]()




&nbsp;
## 2. Hardware assembly

&nbsp;
## 3. Source code

### 3.1 Related work

### 3.2 Source code

&nbsp;
## 4. Run and results

