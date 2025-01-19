#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_

#include <ArduinoLog.h>

#include "mushibot.h"

#define pi 3.1415927

class MotionController
{
public:
    MotionController();
    virtual ~MotionController();

    float loop_pitch();

    Mushibot *p_mushibot;

private:

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
    // LowPassFilter lpf_zeropoint{ .Tf = 0.1 };
    LowPassFilter lpf_zeropoint = LowPassFilter(0.1);
    // LowPassFilter lpf_roll{ .Tf = 0.3 };
    LowPassFilter lpf_roll = LowPassFilter(0.3);


    // LQR自平衡控制器参数
    float LQR_angle = 0;
    float LQR_gyro = 0;
    float LQR_speed = 0;
    float LQR_distance = 0;
    float angle_control = 0;
    float gyro_control = 0;
    float speed_control = 0;
    float distance_control = 0;
    float LQR_u = 0;
    float angle_zeropoint = 2.21;
    float distance_zeropoint = -256.0; // 轮部位移零点偏置（-256为一个不可能的位移值，将其作为未刷新的标志）

    // YAW轴控制数据
    float YAW_gyro = 0;
    float YAW_angle = 0;
    float YAW_angle_last = 0;
    float YAW_angle_total = 0;
    float YAW_angle_zero_point = -10;
    float YAW_output = 0;

    // 腿部舵机控制数据
    byte ID[2];
    s16 Position[2];
    u16 Speed[2];
    byte ACC[2];

    // 逻辑处理标志位
    float robot_speed = 0;         // 记录当前轮部转速
    float robot_speed_last = 0;    // 记录上一时刻的轮部转速
    int jump_flag = 0;             // 跳跃时段标识
    float leg_position_add = 0;    // roll轴平衡控制量
    int uncontrolable = 0;         // 机身倾角过大导致失控

};

#endif


