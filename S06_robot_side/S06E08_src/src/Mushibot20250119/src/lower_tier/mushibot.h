#ifndef MUSHIBOT_H_
#define MUSHIBOT_H_

//机器人控制头文件
#include <Arduino.h>
#include <ArduinoLog.h>

#include <Wire.h>
#include <SimpleFOC.h>
#include <ArduinoJson.h>
#include <MPU6050_tockn.h>
#include "Servo_STS3032.h"

#include <esp_log.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
// #include <esp_adc/adc_continuous.h>


//电量显示LED
#define LED_BAT 13

#define S_RXD 16 // Setting the serial receive pin
#define S_TXD 17 // Setting the serial transmit pin



class Mushibot
{
public:
    Mushibot();
    ~Mushibot();
    void setup_mushibot();
    void loop_mushibot();

    int get_voltage(); 
    void bat_check(); 

    float get_speed(); 
    JsonDocument get_status();

    //电机实例
    BLDCMotor motor1 = BLDCMotor(7);
    BLDCMotor motor2 = BLDCMotor(7);
    BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);
    BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

    //编码器实例
    TwoWire I2Cone = TwoWire(0);
    TwoWire I2Ctwo = TwoWire(1);
    MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
    MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);   

    //STS舵机实例
    SMS_STS sms_sts;

    //MPU6050实例
    MPU6050 mpu6050 = MPU6050(I2Ctwo);   


public:  
    //PID控制器实例
    // PIDController pid_angle{ .P = 1, .I = 0, .D = 0, .ramp = 100000, .limit = 8 };
    PIDController pid_angle = PIDController(1.0, 0.0, 0.0, 100000.0, 8.0);
    // PIDController pid_gyro{ .P = 0.06, .I = 0, .D = 0, .ramp = 100000, .limit = 8 };
    PIDController pid_gyro = PIDController(0.06, 0.0, 0.0, 100000.0, 8.0);
    // PIDController pid_distance{ .P = 0.5, .I = 0, .D = 0, .ramp = 100000, .limit = 8 };
    PIDController pid_distance = PIDController(0.5, 0.0, 0.0, 100000.0, 8.0);
    // PIDController pid_speed{ .P = 0.7, .I = 0, .D = 0, .ramp = 100000, .limit = 8 };
    PIDController pid_speed = PIDController(0.7, 0.0, 0.0, 100000.0, 8.0);
    // PIDController pid_yaw_angle{ .P = 1.0, .I = 0, .D = 0, .ramp = 100000, .limit = 8 };
    PIDController pid_yaw_angle = PIDController(1.0, 0.0, 0.0, 100000.0, 8.0);
    // PIDController pid_yaw_gyro{ .P = 0.04, .I = 0, .D = 0, .ramp = 100000, .limit = 8 };
    PIDController pid_yaw_gyro = PIDController(0.04, 0.0, 0.0, 100000.0, 8.0);
    // PIDController pid_lqr_u{ .P = 1, .I = 15, .D = 0, .ramp = 100000, .limit = 8 };
    PIDController pid_lqr_u = PIDController(1.0, 15.0, 0.0, 100000.0, 8.0);
    // PIDController pid_zeropoint{ .P = 0.002, .I = 0, .D = 0, .ramp = 100000, .limit = 4 };
    PIDController pid_zeropoint = PIDController(0.002, 0.0, 0.0, 100000.0, 4.0);
    // PIDController pid_roll_angle{ .P = 8, .I = 0, .D = 0, .ramp = 100000, .limit = 450 };
    PIDController pid_roll_angle = PIDController(8.0, 0.0, 0.0, 100000.0, 450.0);

    //低通滤波器实例
    // LowPassFilter lpf_joyy{ .Tf = 0.2 };
    LowPassFilter lpf_joyy = LowPassFilter(0.2);
    // LowPassFilter lpf_zeropoint{ .Tf = 0.1 };
    LowPassFilter lpf_zeropoint = LowPassFilter(0.1);
    // LowPassFilter lpf_roll{ .Tf = 0.3 };
    LowPassFilter lpf_roll = LowPassFilter(0.3);

private:  

    //电压检测
    uint16_t bat_check_num = 0;
    int BAT_PIN = 35;  // select the input pin for the ADC

    float speed_prev;
    JsonDocument status_prev;

    void setup_servo();

    // ADC 
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle;
    bool is_calibrated = false;

    void setup_adc();
};

#endif // MUSHIBOT_H_