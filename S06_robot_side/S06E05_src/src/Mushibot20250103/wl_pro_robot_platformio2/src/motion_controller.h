#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_

#include "mushibot.h"
#include "robot.h"
#include <map>

#define pi 3.1415927

class MotionController
{
public:
    MotionController();
    virtual ~MotionController();

    float loop_peace();
    float loop_motion();
    float loop_turn();
    float loop_jump();

    float loop();

    int is_turn() const;
    int is_jump() const;

    void get_params_map(std::map<String, String>& params_map) const;

    Mushibot *p_mushibot;
    
private:
    // PID控制器实例
    PIDController pid_angle      = PIDController(1, 0, 0, 100000, 8);
    PIDController pid_gyro       = PIDController(0.06, 0, 0, 100000, 8);
    PIDController pid_distance   = PIDController(0.5, 0, 0, 100000, 8);
    PIDController pid_speed      = PIDController(0.7, 0, 0, 100000, 8);
    PIDController pid_yaw_angle  = PIDController(1.0, 0, 0, 100000, 8);
    PIDController pid_yaw_gyro   = PIDController(0.04, 0, 0, 100000, 8);
    PIDController pid_lqr_u      = PIDController(1, 15, 0, 100000, 8);
    PIDController pid_zeropoint  = PIDController(0.002, 0, 0, 100000, 4);
    PIDController pid_roll_angle = PIDController(8, 0, 0, 100000, 450);

    // 低通滤波器实例
    LowPassFilter lpf_joyy       = LowPassFilter(0.2);
    LowPassFilter lpf_zeropoint  = LowPassFilter(0.1);
    LowPassFilter lpf_roll       = LowPassFilter(0.3);

#if 0
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

    // 逻辑处理标志位
    float leg_position_add = 0;    // roll轴平衡控制量

#else
    // 根据上报数据更新初始值
    float LQR_angle = 4.37;
    float LQR_distance = 119.87;
    float LQR_gyro = 2.66;
    float LQR_speed = 0.01;
    float LQR_u = -0.02;

    float angle_control = 0.03;
    float gyro_control = 0.16;
    float distance_control = -0.04;
    float speed_control = 0.01;

    float angle_zeropoint = 4.34;
    float distance_zeropoint = 119.94;

    float YAW_angle = 6.30;
    float YAW_angle_last = 6.30;
    float YAW_angle_total = 0.02;
    float YAW_angle_zero_point = 7.66;
    float YAW_gyro = 1.68;
    float YAW_output = 0.08;

    float leg_position_add = 0.76;
#endif

    // 逻辑处理标志位
    int uncontrolable = 0;         // 机身倾角过大导致失控
    int wrobot_move_stop_flag = 0;
    int jump_flag = 0;             // 跳跃时段标识

    // 腿部舵机控制数据
    byte ID[2];
    s16 Position[2];
    u16 Speed[2];
    byte ACC[2];

    void loop_common();
    void loop_all();
    void loop_yaw();
    void loop_leg();
    void loop_jumping();
    void loop_joy();

    void adjust_p_by_height();
    void yaw_angle_addup();

    void get_pid_params_map(std::map<String, String>& params_map, const PIDController& pid, const String& name) const;
};

#endif