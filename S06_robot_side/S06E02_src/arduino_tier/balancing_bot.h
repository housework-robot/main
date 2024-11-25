#include <Arduino.h>
#include <ArduinoJson.h>

#include <SimpleFOC.h>
#include <MPU6050_tockn.h>

#ifndef BALANCING_BOT
#define BALANCING_BOT

class BalancingBot {

  private:
    // The directions of the 2 wheels.
    int M0 = 1;
    int M1 = 1;

    TwoWire I2Cone = TwoWire(0);
    TwoWire I2Ctwo = TwoWire(1);
    MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
    MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
    MPU6050 mpu6050 = MPU6050(I2Ctwo);

    BLDCMotor motor0 = BLDCMotor(7);
    BLDCDriver3PWM driver0 = BLDCDriver3PWM(32,33,25,22);

    BLDCMotor motor1 = BLDCMotor(7);
    BLDCDriver3PWM driver1  = BLDCDriver3PWM(26,27,14,12);

    // control algorithm parametersw
    // stabilisation pid 自稳PID
    //P0.55 I5.5 初始值0.5 5 
    PIDController pid_stb = PIDController(0.8, 0, 0.0005, 100000, 6); 
    // velocity pid 速度PID P初始值1.5
    PIDController pid_vel = PIDController(2, 0, 0.11, 10000, 6);
    // velocity control filtering 速度控制滤波，滤波时间常数为0.07
    LowPassFilter lpf_pitch_cmd = LowPassFilter(0.07);
    // low pass filters for user commands - throttle and steering 油门和转向滤波
    LowPassFilter lpf_throttle = LowPassFilter(0.5);   //初始值0.5
    LowPassFilter lpf_steering = LowPassFilter(0.1);  //初始值0.1

    float steering = 0;
    float throttle = 0;
    float new_steering;
    float new_throttle;

    // max_throttle数值可能需要更改
    float max_throttle = 80; // 初始值20 rad/s 
    float max_steering = 1; //  1V
    float Offset_parameters = -2; //偏置参数

    double acc2rotation(double x, double z);  //for kalman
    void I2C_init();  // I2C初始化

  public:
    void setup();
    JsonDocument get_observation();
    JsonDocument policy(JsonDocument observation, JsonDocument command);
    void step(JsonDocument action);
}; 

#endif