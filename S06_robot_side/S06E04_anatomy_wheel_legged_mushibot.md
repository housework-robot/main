# Anatomy of Mushibot, Another Wheel-legged Robot

# 1. Objectives

In the last article, we studied [Stack-force's wheel-legged robot](https://gitee.com/StackForce/bipedal_wheeled_robot). 
What boards, modules, and chips are used, how are they wired together, and how to write program to access them.

In this article, we studied another wheel-legged robot, [Mushibot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot), 
which is almost a mini version of ETH Zurich's [Ascento](https://www.ascento.ai/), with some differences in the shape and structure of the leg linkages etc. 

In addition to the anatomy of its hardware, we spent more time on Mushibot's motion control. 


&nbsp;
# 2. Hardware

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

**1. BLDC Motor configuration** 

~~~
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
~~~

The `7` here is the pole pair number, referring to the SimpleFOC's tutorial, "[BLDC Motor configuration](https://docs.simplefoc.com/bldcmotor#step-1-creating-the-instance-of-the-bldc-motor)".

**2. BLDC driver 3 PWM**

~~~
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);
~~~

The parameters in `BLDCDriver3PWM()` are the A, B, C phase pwm pins, and the enable pin, referring to the SimpleFOC's tutorial, "[BLDC driver 3 PWM](https://docs.simplefoc.com/bldcdriver3pwm#step-1-hardware-setup)". 

**3. Voltage setting**

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

**4. Torque control**

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





