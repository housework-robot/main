# Anatomy of a Wheel-legged Robot

# 1. Objectives

[Stack-force bipedal wheeled robot](https://gitee.com/StackForce/bipedal_wheeled_robot) is a particially open-sourced project, 
that provides tutorials on the principles, and some source codes. However, it wraps up some lower-level source codes into a library. 

This article dives into the source codes and the related documents of Stack-force bipedal, to get better understanding of this project. 

The top level principle of a wheel-legged robot is quite simple, it uses an [inertial measurement unit (IMU)](https://en.wikipedia.org/wiki/Inertial_measurement_unit) chip 
to get the pose of the robot, including the roll, pitch, and yaw angles, in addition to the forces on x-y-z axes. 
Based on the pose, the robot controls the wheel to move forward, backforward, and make turns, 
controls the legs to stretch and fold, so as to keep the robot standing or moving with balance. 

Therefore, the key to understanding a wheel-legged robot, is to understand how to get robot pose information from IMU, 
to control the motors of the wheels, and to control the servos of legs. 

In details, [Stack-force bipedal wheeled robot](https://gitee.com/StackForce/bipedal_wheeled_robot) uses,

1. Two stack-force self-made BLDC motors, `DengFOC 2208`.

   The two motors are linked to a Stack-force self-made low-power BLDC motor driver board. 
    Their speeds, positions/angles, and torques/currents can be controlled.

2. One `INA240A2` motor current sensor chip.

   The sensor chip is embedded in the Stack-force self-made low-power BLDC motor driver board. 

3. Two `MT6701` motor position sensor modules.

   One `MT6701` sensor for one `DengFOC 2208` motor, they are one-to-one bundled together.

   The two `MT6701` sensors are linked to a Stack-force self-made master control board,
   via [SPI serial communication](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface).

4. Four `DS041MG` servos.

   The four `DS041MG` servos control the four legs.

   They are linked to a Stack-force self-made servo and IMU board, via [I2C serial communication](https://en.wikipedia.org/wiki/I%C2%B2C).

5. One `MPU6050` IMU chip.

   The `MPU6050` IMU chip is embedded in the Stack-force self-made servo and IMU board.

6. Three Stack-force self-made boards.

   A low-power BLDC motor driver board, a master control board, and a servo and IMU board.
   They are stacked together from bottom to top, in the way of Arduino shields.

   The master control board contains two ESP32 chips, `S1` and `S3`.
   `S1` is in charge of controlling the motors, and `S3` for servos and the IMU.

7. Kinematical motion control.

   After collecting robot pose and motion information from `MPU6050` IMU chip, `MT6701` motor position sensor, and `INA240A2` motor current sensor, 
   the robot software system uses algorithms, including inverse kinematics,
   to control the motors and servos, to make the robot moving and keep balance.  

In addition, the Stack-force wheel-legged robot also contains a wireless controller and its receiver module. 
But in this article, we will not discuss the wireless controller and its receiver in details.


# 2. BLDC motors

## 2.1 SimpleFOC motor control code

[Stack-force](https://stackforce.cc/), as known as DengFOC, is based on [the open-source project SimpleFOC](https://docs.simplefoc.com/). 


Following is a sample code from [SimpleFOC official tutorial](https://docs.simplefoc.com/code#step-9-getting-started-step-by-step-guide). 
The content is how to control the BLDC motors with SimpleFOC library. 

~~~
#include <SimpleFOC.h>

// magnetic position sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// external commander interface, for wireless controller.
// Commander command = Commander(Serial);
// void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {
  // monitoring port
  Serial.begin(115200);
  
  // enable the debugging output
  // SimpleFOCDebug::enable(&Serial);

  // initialise magnetic position sensor hardware
  sensor.init();
  // link the motor to the position sensor
  motor.linkSensor(&sensor);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  
  // set control loop type to be used
  motor.controller = MotionControlType::torque;  
  
  // contoller configuration based on the control type 
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 12;  
  
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;
  
  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 50;  
  
  // use monitoring with serial for motor init
  // comment out if not needed
  // motor.useMonitoring(Serial);
  
  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();  
  
  // set the inital target value
  motor.target = 2;
  // define the motor id
  // command.add('A', onMotor, "motor");
  // Run user commands to configure and the motor
  // (find the full command list in docs.simplefoc.com)
  Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));
  
  _delay(1000); 
}

void loop() {
  // iterative setting of the FOC phase voltage
  motor.loopFOC();   
  
  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if target not set in parameter uses motor.target variable
  motor.move();
  
  // user communication
  //command.run();
}  
~~~

As a summary of the SimpleFOC's official tutorial, 
we developers should take the following steps to control the BLDC motors with SimpleFOC library. 

1. Setup BLDC motors.
2. Setup BLDC motor driver, and link the driver to the motors.
3. Setup position sensors, and link the position sensors to the motors.
4. Setup current sensors, and link the current sensors to the motors.
5. Align the motor and sensors, by calling `motor.initFOC()`.
6. Run the loop,
  
   The loop usually consists of
  `motor.loopFOC()` for FOC algorithm execution,
   and `motor.move(target)` for motion control.

&nbsp;
## 2.2 Stack-force BLDC motor control code

Following source code is copied and pasted from [Stack-force toolkit for motor calibration](./S06E03_src/dengfoc_bipedal_bot/BLDC_Control/src/main.cpp). 

~~~
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SF_Motor.h"
#include "SF_Communication.h"
#include "Pins_Specify.h"

//串口发送的

SPIClass vspi(VSPI);//如果用到MT6701编码器，采用的SPI通信，需要实例化SPI总线
TwoWire iic0 = TwoWire(0);//如果是AS5600编码器，采用IIC通信，双电机控制需要实例化两条IIC总线
TwoWire iic1 = TwoWire(1);

SF_Motor M0 = SF_Motor(0);//实例化电机类
SF_Motor M1 = SF_Motor(1);

SF_Communication com = SF_Communication();//实例化通信接口

float Vbus = 12.0;//设置供电电压值
float alignVoltage = 3;//电机-编码器校准时的电压值

void setup()
{
    Serial.begin(115200);
    com.linkMotor(M0, M1); // 链接电机和通信接口 可以选一个或两个
    com.init(ONBOARD); //选择通信的输出 USB:通过USB输出；ONBOARD:与扩展主控通信。
    // com.init(USB);

    //为AS5600编码器的初始化
    // iic0.begin(AS5600_SDA0, AS5600_SCL0, 400000UL);
    // iic1.begin(AS5600_SDA1, AS5600_SCL1, 400000UL);
    // M0.initEncoder(AS5600,iic0);
    // M1.initEncoder(AS5600,iic1);
    //为MT6701编码器的初始化
    vspi.begin(MT6701_CLK, MT6701_DO, 0, -1);
    M0.initEncoder(MT6701, vspi);
    M1.initEncoder(MT6701, vspi);

    //电机初始化，传入电压值
    M0.init(Vbus);
    M1.init(Vbus);
    //电机-编码器的校准，传入校准时的电压值
    M0.AlignSensor(alignVoltage);
    M1.AlignSensor(alignVoltage);

    //通信端口开启持续的电机状态输出
    com.start();

    //设置相关的PID值 传入参数为P、I、D、Limit值
    M0.setAnglePID(0.5, 0, 0, 6);
    M0.setVelPID(0.05, 0.005, 0, 6);
    M0.setCurrentPID(1.2,0,0,0);

    M1.setAnglePID(0.5, 0, 0, 6);
    M1.setVelPID(0.05, 0.005, 0, 6);
    M1.setCurrentPID(1.2,0,0,0);
    
}

uint32_t now_time = 0;
uint32_t last_time = 0;

void loop()
{
  //电机的循环控制执行
  M0.run();
  M1.run();
  

  //设置电机的控制方式示例,当然也可以通过串口控制
  M0.setTorque(1);
  M1.setTorque(1);
  M0.setForceAngle(0);
  M1.setForceAngle(0);
  
  M0.setVelocity(10);
  M1.setVelocity(10);
}
~~~

Comparing Stack-force's motor control with SimpleFOC's, 

1. Setup BLDC motors.
   
   *Both are the same.*
   
2. Setup BLDC motor driver, and link the driver to the motors.
   
   *Stack-force doesn't setup the driver explicitly, maybe the driver is embedded inside Stack-force's library.*
   
3. Setup position sensors, and link the position sensors to the motors.
   
   *Both are the same.*
   
4. Setup current sensors, and link the current sensors to the motors.
   
   *Stack-force doesn't setup the current sensors explicitly, but it can get the current value,
   maybe the current sensor is embedded inside Stack-force's library.*
   
5. Align the motor and sensors, by calling `motor.initFOC()`.

   *Stack-force uses M0.AlignSensor(alignVoltage) to align the motor with sensor,
   similar but not exactly the same with SimpleFOC.*
   
6. Run the loop,
  
   The loop usually consists of
  `motor.loopFOC()` for FOC algorithm execution,
   and `motor.move(target)` for motion control.

   *Stack-force uses M0.run() for the loop, without explicity calling loopFOC(),
   similar but not exactly the same with SimpleFOC.*

In summary, Stack-force's motor control is consistent with the standord SimpleFOC's routine, 
except it hides some details, including wrapping up the motor driver in Stack-force's library. 


&nbsp;
## 2.3 SF_Motor library and PWM driver

Hardware wiring

PWM code, https://dengfoc.com/#/dengfoc/%E7%81%AF%E5%93%A5%E6%89%8B%E6%8A%8A%E6%89%8B%E6%95%99%E4%BD%A0%E5%86%99FOC%E7%AE%97%E6%B3%95/4.2FOC%E5%BC%80%E7%8E%AF%E9%80%9F%E5%BA%A6%E4%BB%A3%E7%A0%81%E7%9A%84%E6%92%B0%E5%86%99


&nbsp;
## 2.4 SF_Motor serial communication

ONBOARD uart:  hardware wiring
Serial USB

&nbsp;
## 2.5 MT6701 motor sensors

1. MT6701 position sensor: SPI
    hardware wiring
   
3. INA240A2 current sensor: inline


&nbsp;
## 2.6 SF_BLDC library and 4 control modes

4 modes,

