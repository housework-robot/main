# Programming 2-wheel Balancing Bot

by Kan Deng, Yujie Wang, Yaoxuan Wang, Nov. 16, 2024

----------------------------------------------------------

## 1. Objectives

[Arduino SimpleFOCBalancer](https://github.com/simplefoc/Arduino-FOC-balancer) is an open source project that builds a two wheel balancing robot 
based on 2 BLDC motors, 1 [MPU6050 IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit), 1 ESP32 module for Bluetooth communication, 
and [SimpleFOC library](https://github.com/simplefoc).

[Balance_Bot_DengFOC](https://github.com/ToanTech/Balance_Bot_DengFOC) simplifies simpleFOC by focusing on a few selected chip modules and boards. 
In addition it provides more detailed tutorials and cheapers electrical and mechanical components. 

However [the source code of Balance_Bot_DengFOC](https://github.com/ToanTech/Balance_Bot_DengFOC/blob/main/DengFOC%20%E5%B9%B3%E8%A1%A1%E8%BD%A6%E7%A8%8B%E5%BA%8F/Wx_BlueToothBalancer/Wx_BlueToothBalancer.ino) is for windows. 
To migrate it from windows to ubuntu, there are a few technical challenges. 

This document records the steps we took to migrate `Balance_Bot_DengFOC` source code from windows to ubuntu. 

&nbsp;
## 2. Arduino libraries

To migrate `Balance_Bot_DengFOC` from windows to ubuntu, we have to manually install the related libraries in Arduino IDE.  

### 2.1 Network proxy

Before installing the libraries, we need to setup the network proxy, to make it convenient to download the libraries especially those resides in Github. 

The challenge is that there are only a few VPNs available for ubuntu, e.g. [lantern.io](https://lantern.io/), and they are not stable and quite slow for downloading. 

Our solution is to use two computer, one is Ubuntu, the other is either Macbook or Windows. 

#### 1. Find out the IP address and proxy port of the Macbook.

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


#### 2. Configure the proxy of the Ubuntu.

   Open ubuntu's `System Settings`, navigate to `Network`, and configure it using Macbook's IP address and proxy port, shown in the following screen snapshots.

   <p align="center">
     <img alt="Ubuntu network setting" src="./S06E01_src/ubuntu_proxy01.png" width="48%">
     &nbsp;  
     <img alt="Ubuntu proxy" src="./S06E01_src/ubuntu_proxy02.png" width="48%">
   </p>


&nbsp;
### 2.2 ESP32

#### 1. Additional Board Manager URLs

Following [the official installation guide of ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html), 
fill in the `Stable release link` in the `Additional Board Manager URLs` field in Arduino IDE's preference window.

After then, restart the Arduino IDE, you will see the various ESP32 boards listed in the Arduino IDE. 

   <p align="center">
     <img alt="ESP32 board link" src="./S06E01_src/ESP32_setting.png" width="48%">
     &nbsp;  
     <img alt="ESP32 board list" src="./S06E01_src/ESP32_boards.png" width="48%">
   </p>


#### 2. Install ESP32 libraries

With different board, there are related libraries. To install ESP32 libraries, we selected `Arduino Nano ESP32` board. Then, we opened the `board manager` and search `esp32`. 

We installed the ESP32 libraries related to the `Arduino Nano ESP32` board, one by one. 

The latest version of `esp32` library is `3.0.5`, but for unknown reason, we failed to install `esp 3.0.5`. Hence, we installed an earlier version, `3.0.3`. 

To check the success of ESP32 library installation, we can open the examples, if successful, we will see many examples related to ESP32. 

   <p align="center">
     <img alt="ESP32 library installation" src="./S06E01_src/ESP32_library01.png" width="48%">
     &nbsp;  
     <img alt="ESP32 examples" src="./S06E01_src/ESP32_library02.png" width="48%">
   </p>



&nbsp;
### 2.3 I2CDev

[The I2C Device Library (i2cdevlib)](https://github.com/jrowberg/i2cdevlib) is a collection of libraries to provide simple and intuitive interfaces to I2C devices.

Following its installation guide, 

1. Find the Arduino IDE library file directory, by clicking Arduino IDE's `File` > `Preference`,

   then you will see the Arduino IDE library file directory in the Preference window.

   <p align="center">
     <img alt="Arduino IDE library file directory" src="./S06E01_src/Arduino_library_directory.png" width="50%">
   </p>
   

2. We downloaded a .zip archive of [the I2C Device github repo](https://github.com/jrowberg/i2cdevlib).

3. Copied two relevant libraries, [I2CDev](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev) and [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050), into our Arduino IDE library subfolder.

   ~~~
   $ cd /home/robot/Software/
   $ unzip i2cdevlib-master.zip
   $ cd /home/robot/Software/i2cdevlib-master/Arduino

   $ cp -r I2Cdev/  /home/robot/Arduino/libraries/.
   $ cp -r MPU6050 /home/robot/Arduino/libraries/.
   $ ls /home/robot/Arduino/libraries/
     Adafruit_Circuit_Playground  Bridge    Firmata  Keyboard       MPU6050        RobotIRremote  Servo          SimpleFOCDrivers  Temboo
     Arduino_ESP32_OTA            Esplora   GSM      LiquidCrystal  MPU6050_tockn  Robot_Motor    SimpleDCMotor  SpacebrewYun      TFT
     ArduinoHttpClient            Ethernet  I2Cdev   Mouse          Robot_Control  SD             Simple_FOC     Stepper
   ~~~

   Notice that we may not need `MPU6050` library, but it doesn't hurt to install redundant libraries. 

4. Restart Arduino IDE, then we will see those two I2C libraries. 

   <p align="center">
     <img alt="Installed I2C libraries in the Arduino IDE" src="./S06E01_src/I2C_library.png" width="50%">
   </p>
   

&nbsp;
### 2.4 MPU6050_tockn

1. Download a .zip archive of [the MPU6050_tockn's github repo](https://github.com/tockn/MPU6050_tockn)

2. Open Arduino IDE, and click `Sketch` > `Include Library` > `Add .ZIP Library`, then select the downloaded `MPU6050_tockn-master.zip` github archive.

   <p align="center">
     <img alt="Add MPU6050_tockn zip in the Arduino IDE" src="./S06E01_src/MPU6050_add_zip.png" width="48%">
     &nbsp; 
     <img alt="Select MPU6050_tockn-master.zip as the library source" src="./S06E01_src/MPU6050_file_selection.png" width="48%">
   </p>

3. Restart Arduino IDE, then we will see the `MPU6050_tockn` library. 



&nbsp;
### 2.5 SimpleFOC

Following [the SimpleFOC official installation guide](https://docs.simplefoc.com/library_download), 

1. Click the `library` icon on the left side bar on the Arduino IDE.

2. Search for `simplefoc`, and install the latest versions of `Simple FOC`, `SimpleDCMotor`, and `SimpleFOCDrivers`.

   <p align="center">
     <img alt="Click the library icon on the left side bar" src="./S06E01_src/simplefoc_navigation.png" width="48%">
     &nbsp; 
     <img alt="search for simplefoc libraries and install them" src="./S06E01_src/simplefoc_search_install.png" width="48%">
   </p>   


&nbsp;
## 3. Software

Since we use a ubuntu computer, and install an Arduino IDE from scratch, the arduino compiler might be different from that one 
used by [Balance_Bot_DengFOC](https://github.com/ToanTech/Balance_Bot_DengFOC)'s author. 

When compiling we encountered some minor error, mainly syntax error. We modified the original source code, and compiled successfully. Our modification refers to the following,

~~~
// velocity pid 速度PID P初始值1.5
// PIDController pid_vel{.P = 2, .I = 0, .D = 0.11, .ramp = 10000, .limit = 6};
PIDController pid_vel(2, 0, 0.11, 10000, 6);

// velocity control filtering 速度控制滤波，滤波时间常数为0.07
// LowPassFilter lpf_pitch_cmd{.Tf = 0.07};
LowPassFilter lpf_pitch_cmd(0.07);

// low pass filters for user commands - throttle and steering 油门和转向滤波
// LowPassFilter lpf_throttle{.Tf = 0.5}; //初始值0.5
LowPassFilter lpf_throttle(0.5); //初始值0.5
// LowPassFilter lpf_steering{.Tf = 0.1}; //初始值0.1
LowPassFilter lpf_steering(0.1);

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // std::string rxValue = pCharacteristic->getValue();
      String rxValue = pCharacteristic->getValue();
~~~


[Our source code](./S06E01_src/balancing_bot.ino) has been uploaded onto this repo.


&nbsp;
## 4. Electronics assembly

We used a white USB cable connecting our ubuntu computer to the ESP32 module, referring to the white cable in the left image below. 

Inside the white box, there is a board similar to Arduino, that is self-made by the author of [Balance_Bot_DengFOC](https://github.com/ToanTech/Balance_Bot_DengFOC). 

More specifically, the ESP32 module is `WEMOS LOLIN32 Lite`. It is used as a shield on top of DengFOC board. 

Therefore, even though we only connect to the ESP32 module, but also we can access the underneath DengFOC board. 

   <p align="center">
     <img alt="USB cable connects a computer to the ESP32 module" src="./S06E01_src/balancing_bot_wiring.jpg" width="48%">
     &nbsp; 
     <img alt="DengFOC board pinout" src="./S06E01_src/DengFOC_board.png" width="48%">
   </p>  

More details of the electronics assembly refers to [the github repo of DengFOC](https://github.com/ToanTech/Balance_Bot_DengFOC), 
which provides a step-by-step guide for the assembly. 


&nbsp;
## 5. Loading from Arduino IDE

After connecting our ubuntu computer, where an Arduino IDE is running, to the balancing bot using a USB cable, now it is time to the load our source code to the bot. 

One detail is that the baud rate must be `115200`, because in the source code, we set the Serial to be with `115200` baud rate. 

~~~
void setup(){
    Serial.begin(115200);
~~~

   <p align="center">
     <img alt="USB cable connects a computer to the ESP32 module" src="./S06E01_src/baud_rate.png" width="50%">
   </p>  


&nbsp;
## 5 Demo video

Click the following image and display a video hosted in Youtube. 

   [![Setup Arduino IDE to program DengFOC's balancing bot](https://img.youtube.com/vi/WwpEb_d7BzI/hqdefault.jpg)](https://www.youtube.com/watch?v=WwpEb_d7BzI)

