# Anatomy of Mushibot, Another Wheel-legged Robot

# 1. Objectives

In the last article, we studied [Stack-force's wheel-legged robot](https://gitee.com/StackForce/bipedal_wheeled_robot). 
What boards, modules, and chips are used, how are they wired together, and how to write program to access them.

In this article, we studied another wheel-legged robot, [Mushibot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot), 
which is almost a mini version of ETH Zurich's [Ascento](https://www.ascento.ai/), with some differences in the shape and structure of the leg linkages etc. 

In addition to the anatomy of its hardware, we spent more time on Mushibot's motion control. 


&nbsp;
# 2. Actuators and Sensors

Let's start with the hardware used in the Mushibot, especially

1. Two 2208 80T-100kv BLDC motors that control the wheels.
2. Two AS5600-ASOM motor encoders that monitor the rotation speed of the motors.
3. Two Feetech's STS3032 servos that control the pose of the legs.
4. One MPU6050 IMU module.

We will study how those hardware are wired to the master controller board, how to write C/C++ program to access them. 

## 2.1 2208 BLDC motors

### 2.1.1 Software 

The following code snippet is extracted from Mushibot's `wl_pro_robot/wl_pro_robot.ino`, which is the main program of the Mushibot system. 
In the code, we can learn, 
1. how to set up the motors with drivers,
2. how to initialize and start the motors,
3. and how to control the torques of the motors. 

~~~
// /Users/dengkan/Projects/Mushibo-wheel-legged-robot/3.Software/wl_pro_robot/wl_pro_robot.ino

// 机器人控制头文件
#include <SimpleFOC.h>
#include <Arduino.h>

//电机实例
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);

void setup() {
  Serial.begin(115200);  //通讯串口

  // 驱动器设置
  // set motor's power supply voltage,
  // the rule of thumb is to align with the motor.voltage_limit for the open loop.
  motor1.voltage_sensor_align = 6;
  motor2.voltage_sensor_align = 6;

  // set driver's power supply voltage [V]
  driver1.voltage_power_supply = 8;
  driver2.voltage_power_supply = 8;

  driver1.init();
  driver2.init();

  // 连接motor对象与驱动器对象
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  // set the torque control type
  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;

  // set motion control loop to be used
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
  
  // monitor相关设置
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // 电机初始化
  // initialize motor
  motor1.init();
  // align sensor and start FOC
  motor1.initFOC();

  motor2.init();
  motor2.initFOC();

  delay(500);
}

void loop() {
  lqr_balance_loop(); // lqr自平衡控制，calculate 'LQR_u'
  yaw_loop();         // yaw轴转向控制, calculate 'YAW_output'
  
  //将自平衡计算输出转矩赋给电机
  motor1.target = (-0.5)*(LQR_u + YAW_output);
  motor2.target = (-0.5)*(LQR_u - YAW_output);
  ...
  
  //迭代计算FOC相电压
  motor1.loopFOC();
  motor2.loopFOC();
  
  //设置轮部电机输出
  motor1.move();
  motor2.move();
}
~~~

Let's dive into the code. 

#### 1. BLDC Motor configuration

~~~
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
~~~

The `7` here is the pole pair number, referring to the SimpleFOC's tutorial, "[BLDC Motor configuration](https://docs.simplefoc.com/bldcmotor#step-1-creating-the-instance-of-the-bldc-motor)".

#### 2. BLDC driver 3 PWM

~~~
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);
~~~

The parameters in `BLDCDriver3PWM()` are the A, B, C phase pwm pins, and the enable pin, referring to the SimpleFOC's tutorial, "[BLDC driver 3 PWM](https://docs.simplefoc.com/bldcdriver3pwm#step-1-hardware-setup)". 

#### 3. Voltage setting

~~~
void setup() {
  // 驱动器设置
  // set motor's power supply voltage [V],
  // the rule of thumb is to align with the motor.voltage_limit for the open loop.
  motor1.voltage_sensor_align = 6;
  motor2.voltage_sensor_align = 6;

  // set driver's power supply voltage [V]
  driver1.voltage_power_supply = 8;
  driver2.voltage_power_supply = 8;
}
~~~

It is a bit tricky to set the power supply's voltages of the motors and drivers, because they must be aligned with the voltages of the motor's encoders. 

A high-level guidance refers to the SimpleFOC's tutorial, "[Let’s get started](https://docs.simplefoc.com/example_from_scratch#step-3-closed-loop-control---torque-using-voltage)", especially "Step 3. Closed-loop control - torque using voltage" section, and the example sketch for "BLDC Motor + 3PWM driver + Encoder".

A more detailed explanation refers to SimpleFOC's tutorial, "[Torque control using voltage](https://docs.simplefoc.com/voltage_torque_mode)"。

#### 4. Torque control

~~~
void setup() {
  // set the torque control type
  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;

  // set motion control type to be used
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
}

void loop() {
  motor1.target = (-0.5)*(LQR_u + YAW_output);
  motor2.target = (-0.5)*(LQR_u - YAW_output);
}
~~~

Mushibot set the torque of the motor to be controlled by the power supply's voltage. Also, it set the motor's control mode to be torque. 

Therefore, we can control the motors by changing their power supply's voltages. 

&nbsp;
### 2.1.2 Hardware

   <p align="center">
     <img alt="the partial Mushibot schematic of the master controller board for motor controlling" src="./S06E04_src/images/Mushibot_master_board_motor.png" width="90%">
   </p>

#### 1. ESP32-WROOM-32 chip

The schematic on the left is a ESP32-WROOM-32 chip with its pins. 

Overthere, you can find `IO32` `IO33` `IO25` `IO22` and `IO26` `IO27` `IO14` `IO12` 
correspond to the parameters of `BLDCDriver3PWM()` of the two motors. 

~~~
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);
~~~

#### 2. Battery ADC

The `BAT_ADC` in the upper-middle schematic refers to the Analog-to-Digital Converter to control the voltage of the power supply. 

Mushibot doesn't have a separated board for the motor PWM drivers. Instead, it controls the motors by the master controller board directly. 

#### 3. HDR-M-2.54

The lower-middle schematic is for HDR-M-2.54, a pin header male connector, with 2.54mm pin spacing.

The motors are wired to the HDR-M-2.54 pin connectors, J1 and J3. 

The upper-right schematic is also for HDR-M-2.54, but it is for the motor encoders, which we will discuss in next section. 

&nbsp;
## 2.2 AS5600-ASOM motor encoder

The following code snippet is extracted from Mushibot's `wl_pro_robot/wl_pro_robot.ino`, which is the main program of the Mushibot system. In the code, we can learn,

1. how to set up the motor encoders with I2C serial connection,
2. how to initialize and start the motor encoders,
3. and how to read the motor's rotation angle and velocity from the encoders.

~~~
// /Users/dengkan/Projects/Mushibo-wheel-legged-robot/3.Software/wl_pro_robot/wl_pro_robot.ino

//机器人控制头文件
#include <SimpleFOC.h>
#include <Arduino.h>

//电机实例
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);

//编码器实例
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);


void setup() {
  // 编码器设置
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  
  //连接motor对象与编码器对象
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  //连接motor对象与驱动器对象
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  delay(500);
}

void loop() {
  lqr_balance_loop(); //lqr自平衡控制
  yaw_loop();         //yaw轴转向控制

  //将自平衡计算输出转矩赋给电机
  motor1.target = (-0.5)*(LQR_u + YAW_output);
  motor2.target = (-0.5)*(LQR_u - YAW_output);
 
  //迭代计算FOC相电压
  motor1.loopFOC();
  motor2.loopFOC();
  
  //设置轮部电机输出
  motor1.move();
  motor2.move();
}

//lqr自平衡控制
void lqr_balance_loop(){
  //给负值是因为按照当前的电机接线，正转矩会向后转
  LQR_distance  = (-0.5) *(motor1.shaft_angle + motor2.shaft_angle);
  LQR_speed     = (-0.5) *(motor1.shaft_velocity + motor2.shaft_velocity);
  ...
}
~~~

Let's dive into the code. 

### 2.2.1 Encoder configuration

As a full-fledged configuration, we need to create an instance of the configuration first,

~~~
// Data structure of the magnetic sensor I2C configuration
struct MagneticSensorI2CConfig_s  {
  int chip_address;
  int bit_resolution; 
  int angle_register;
  int data_start_bit; 
};

// configuration for AS5600 sensor
MagneticSensorI2CConfig_s MySensorConfig = {
  .chip_address = 0x36, 
  .bit_resolution = 12, 
  .angle_register=0x0E, 
  .data_start_bit=11
}; 
~~~

After then provide it to the constructor, 

~~~
// the sensor class with desired sensor configuration
MagneticSensorI2C sensor = MagneticSensorI2C(MySensorConfig);

void setup(){
  sensor.init();
  ...
}
~~~

For the most common I2C magnetic sensors, the  SimpleFOC library provides the simplified configuration constructor. 
Namely for `AS5600` 12-bit sensor and `AS5048` 14-bit sensor. 

More detail refers to the SimpleFOC tutorial, "[I2C Magnetic sensor setup](https://docs.simplefoc.com/magnetic_sensor_i2c#quick-configuration-for-common-sensors)".

~~~
#include <SimpleFOC.h>

MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
~~~


### 2.2.2 I2C communication 

Mushibot uses I2C serial communication to link the encoders to the motors. 

~~~
//编码器实例
// 0 and 1 are sequence IDs, to distinguish themself from others. 
TwoWire I2Cone = TwoWire(0);  
TwoWire I2Ctwo = TwoWire(1);  

void setup() {
  // 编码器设置
  // bool begin(int sdaPin, int sclPin, uint32_t frequency);
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL);

  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  
  //连接motor对象与编码器对象
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
}
~~~

   <p align="center">
     <img alt="the partial Mushibot schematic of the master controller board for motor controlling" src="./S06E04_src/images/Mushibot_master_board_motor.png" width="90%">
   </p>

In the schematic diagram on the left side, we can find the `IO19` `IO18` and `IO23` `IO5` pins, correspond to the following code. 

~~~
  // 编码器设置
  // bool begin(int sdaPin, int sclPin, uint32_t frequency);
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL);
~~~

The upper-right schematic is for HDR-M-2.54_1x4, a pin header male connector, one column of four pins, with 2.54mm pin spacing.

`J2` `J4` pin connectors are for the encoders' I2C serial communication. 

`I2Cone.begin(int sdaPin, int sclPin, uint32_t frequency)` is an I2C function, and this function is used only for I2C's master mode, 
referring to [ESP32's API documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html#i2c-master-apis).

   <p align="center">
     <img alt="the stack of stack-force boards" src="./S06E04_src/images/arduino_i2c_master.png" width="48%">
     &nbsp;  
     <img alt="the outlook of the motor driver board" src="./S06E04_src/images/arduino_i2c_slave.png" width="48%">
   </p>

The above diagrams illustrate the difference between I2C's master mode and slave mode. 

An ESP32 chip can be used either as a master or as a slave. The diagram on the left is an ESP32 chip behaves as a master communicating with multiple slave devices. 

There are two ESP32 chips in the diagram on the right, one for the master, the other for the slave, they communicate with each other via I2C. 


### 2.2.3 Read from encoder

It is quite straightforward to read motor's rotation angle and velocity from the encoder. 

In Mushibot system, the following code gives an example of the usage,
simply retrieving the values of motor's member variables `motor1.shaft_angle` and `motor1.shaft_velocity`.  

~~~
void lqr_balance_loop(){
  //给负值是因为按照当前的电机接线，正转矩会向后转
  LQR_distance  = (-0.5) *(motor1.shaft_angle + motor2.shaft_angle);
  LQR_speed     = (-0.5) *(motor1.shaft_velocity + motor2.shaft_velocity);
  ...
}
~~~

&nbsp;
### 2.2.2 Hardware

&nbsp;
## 2.3 STS3032 servo

### 2.3.1 Software
### 2.3.2 Hardware

&nbsp;
## 2.4 MPU6050 IMU module

### 2.4.1 Software
### 2.4.2 Hardware


&nbsp;
# 3. Motion control

