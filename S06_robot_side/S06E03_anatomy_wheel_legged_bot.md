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

## 2.1 SimpleFOC motor control routine

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

   
