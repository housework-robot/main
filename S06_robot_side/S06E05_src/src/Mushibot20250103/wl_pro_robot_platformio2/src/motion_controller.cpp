#include "motion_controller.h"

MotionController::MotionController()
{
    ID[0] = 1;
    ID[1] = 2;
    
    ACC[0] = 8;
    ACC[1] = 8;
    
    Speed[0] = 200;
    Speed[1] = 200;
}

MotionController::~MotionController()
{

}

float MotionController::loop()
{
    // LQR平衡算式，实际使用中为便于调参，讲算式分解为4个P控制，采用PIDController方法在commander中实时调试
    // QR_u = LQR_k1*(LQR_angle - angle_zeropoint) + LQR_k2*LQR_gyro + LQR_k3*(LQR_distance - distance_zeropoint) + LQR_k4*LQR_speed;

    // 给负值是因为按照当前的电机接线，正转矩会向后转
    LQR_distance = (-0.5) * (p_mushibot->motor1.shaft_angle + p_mushibot->motor2.shaft_angle);
    // LQR_speed = (-0.5) * (motor1.shaft_velocity + motor2.shaft_velocity);
    LQR_speed = p_mushibot->get_speed();
    LQR_angle = (float)p_mushibot->mpu6050.getAngleY();
    LQR_gyro = (float)p_mushibot->mpu6050.getGyroY();
    // Serial.println(LQR_distance);

    // Serial.println(LQR_angle);
    // Serial.print("--->");
    // Serial.println(angle_control);

    // 计算自平衡输出
    angle_control = pid_angle(LQR_angle - angle_zeropoint);
    gyro_control = pid_gyro(LQR_gyro);

    // 运动细节优化处理
    if (wrobot.joyy != 0) // 有前后方向运动指令时的处理
    {
        distance_zeropoint = LQR_distance; // 位移零点重置
        pid_lqr_u.error_prev = 0;          // 输出积分清零
    }

    if ((wrobot.joyx_last != 0 && wrobot.joyx == 0) || (wrobot.joyy_last != 0 && wrobot.joyy == 0)) // 运动指令复零时的原地停车处理
    {
        wrobot_move_stop_flag = 1;
    }
    if ((wrobot_move_stop_flag == 1) && (abs(LQR_speed) < 0.5))
    {
        distance_zeropoint = LQR_distance; // 位移零点重置
        wrobot_move_stop_flag = 0;
    }

    if (abs(LQR_speed) > 15) // 被快速推动时的原地停车处理
    {
        distance_zeropoint = LQR_distance; // 位移零点重置
    }

    // 计算位移控制输出
    distance_control = pid_distance(LQR_distance - distance_zeropoint);
    speed_control = pid_speed(LQR_speed - 0.1 * lpf_joyy(wrobot.joyy));

    // 轮部离地检测
    if (abs(p_mushibot->get_speed() - p_mushibot->get_last_speed()) > 10 || abs(p_mushibot->get_speed()) > 50 || (jump_flag != 0)) // 若轮部角速度、角加速度过大或处于跳跃后的恢复时期，认为出现轮部离地现象，需要特殊处理
    {
        distance_zeropoint = LQR_distance;    // 位移零点重置
        LQR_u = angle_control + gyro_control; // 轮部离地情况下，对轮部分量不输出；反之，正常状态下完整输出平衡转矩
        pid_lqr_u.error_prev = 0;             // 输出积分清零
    }
    else
    {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

    // 触发条件：遥控器无信号输入、轮部位移控制正常介入、不处于跳跃后的恢复时期
    if (abs(LQR_u) < 5 && wrobot.joyy == 0 && abs(distance_control) < 4 && (jump_flag == 0))
    {

        LQR_u = pid_lqr_u(LQR_u); // 补偿小转矩非线性
        // Serial.println(LQR_u);
        angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control)); // 重心自适应
    }
    else
    {
        pid_lqr_u.error_prev = 0; // 输出积分清零
    }

    loop_all();
    return LQR_u;
}

void MotionController::loop_common()
{
    // 给负值是因为按照当前的电机接线，正转矩会向后转
    LQR_distance = (-0.5) * (p_mushibot->motor1.shaft_angle + p_mushibot->motor2.shaft_angle);
    LQR_speed = p_mushibot->get_speed();
    LQR_angle = (float) p_mushibot->mpu6050.getAngleY();
    LQR_gyro  = (float) p_mushibot->mpu6050.getGyroY();

    // 计算自平衡输出
    angle_control = pid_angle(LQR_angle - angle_zeropoint);
    gyro_control  = pid_gyro(LQR_gyro);
    speed_control = pid_speed(LQR_speed);
}

void MotionController::loop_all()
{
    adjust_p_by_height();
    loop_yaw();
    loop_jumping();
    loop_leg();

    float weight = -0.5;
    p_mushibot->motor1.target = weight * (LQR_u + YAW_output);
    p_mushibot->motor2.target = weight * (LQR_u - YAW_output);

    loop_joy();
}

float MotionController::loop_peace()
{
    loop_common();

    // 计算位移控制输出
    distance_control = pid_distance(LQR_distance - distance_zeropoint);
    LQR_u = angle_control + gyro_control + distance_control + speed_control;
        
    // 触发条件：遥控器无信号输入、轮部位移控制正常介入、不处于跳跃后的恢复时期
    if (abs(LQR_u) < 5 && abs(distance_control) < 4)
    {

        LQR_u = pid_lqr_u(LQR_u); // 补偿小转矩非线性
        angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control)); // 重心自适应
    }
    else
    {
        pid_lqr_u.error_prev = 0; // 输出积分清零
    }
    loop_all();
    return LQR_u;
}

float MotionController::loop_motion()
{
    loop_common();

    distance_zeropoint = LQR_distance; // 位移零点重置
    // 计算位移控制输出
    distance_control = pid_distance(LQR_distance - distance_zeropoint);

    if (abs(p_mushibot->get_speed() - p_mushibot->get_last_speed()) > 10 || 
        abs(p_mushibot->get_speed()) > 50) // 若轮部角速度、角加速度过大或处于跳跃后的恢复时期，认为出现轮部离地现象，需要特殊处理
    {
        distance_zeropoint = LQR_distance;    // 位移零点重置
        LQR_u = angle_control + gyro_control; // 轮部离地情况下，对轮部分量不输出；反之，正常状态下完整输出平衡转矩
        pid_lqr_u.error_prev = 0;             // 输出积分清零
    }
    else
    {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

        // 触发条件：轮部位移控制正常介入、不处于跳跃后的恢复时期
    // 不理解这段话的逻辑 2024-12-26
    if (abs(LQR_u) < 5 && abs(distance_control) < 4)
    {
        LQR_u = pid_lqr_u(LQR_u); // 补偿小转矩非线性
        angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control)); // 重心自适应
    }
    else
    {
        pid_lqr_u.error_prev = 0; // 输出积分清零
    }

    loop_all();
    return LQR_u;
}

float MotionController::loop_jump()
{
    loop_common();

    if (abs(LQR_speed) > 15) // 被快速推动时的原地停车处理
    {
        distance_zeropoint = LQR_distance; // 位移零点重置
    }

    // 计算位移控制输出
    distance_control = pid_distance(LQR_distance - distance_zeropoint);

    if (abs(p_mushibot->get_speed() - p_mushibot->get_last_speed()) > 10 || 
        abs(p_mushibot->get_speed()) > 50 || (jump_flag != 0)) // 若轮部角速度、角加速度过大或处于跳跃后的恢复时期，认为出现轮部离地现象，需要特殊处理
    {
        distance_zeropoint = LQR_distance;    // 位移零点重置
        LQR_u = angle_control + gyro_control; // 轮部离地情况下，对轮部分量不输出；反之，正常状态下完整输出平衡转矩
        pid_lqr_u.error_prev = 0;             // 输出积分清零
    }
    else
    {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

    pid_lqr_u.error_prev = 0; // 输出积分清零

    loop_all();
    return LQR_u;
}

float MotionController::loop_turn()
{
    loop_common();

    // 运动细节优化处理
    if (wrobot.joyy != 0) // 有前后方向运动指令时的处理
    {
        distance_zeropoint = LQR_distance; // 位移零点重置
        pid_lqr_u.error_prev = 0;          // 输出积分清零
    }

    if ((wrobot.joyx_last != 0 && wrobot.joyx == 0) || 
        (wrobot.joyy_last != 0 && wrobot.joyy == 0)) // 运动指令复零时的原地停车处理
    {
        wrobot_move_stop_flag = 1;
    }
    if ((wrobot_move_stop_flag == 1) && (abs(LQR_speed) < 0.5))
    {
        distance_zeropoint = LQR_distance; // 位移零点重置
        wrobot_move_stop_flag = 0;
    }

    if (abs(LQR_speed) > 15) // 被快速推动时的原地停车处理
    {
        distance_zeropoint = LQR_distance; // 位移零点重置
    }

    // 计算位移控制输出
    distance_control = pid_distance(LQR_distance - distance_zeropoint);

    if (abs(p_mushibot->get_speed() - p_mushibot->get_last_speed()) > 10 || 
        abs(p_mushibot->get_speed()) > 50) // 若轮部角速度、角加速度过大或处于跳跃后的恢复时期，认为出现轮部离地现象，需要特殊处理
    {
        distance_zeropoint = LQR_distance;    // 位移零点重置
        LQR_u = angle_control + gyro_control; // 轮部离地情况下，对轮部分量不输出；反之，正常状态下完整输出平衡转矩
        pid_lqr_u.error_prev = 0;             // 输出积分清零
    }
    else
    {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

    // 触发条件：遥控器无信号输入、轮部位移控制正常介入、不处于跳跃后的恢复时期
    if (abs(LQR_u) < 5 && wrobot.joyy == 0 && abs(distance_control) < 4)
    {

        LQR_u = pid_lqr_u(LQR_u); // 补偿小转矩非线性
        angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control)); // 重心自适应
    }
    else
    {
        pid_lqr_u.error_prev = 0; // 输出积分清零
    }    

    loop_all();
    return LQR_u;
}

void MotionController::loop_yaw()
{
    yaw_angle_addup();

    YAW_angle_total += wrobot.joyx * 0.002;
    float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
    float yaw_gyro_control = pid_yaw_gyro(YAW_gyro);
    YAW_output = yaw_angle_control + yaw_gyro_control;    
}

void MotionController::loop_leg()
{
    if (0 != jump_flag)
    {
        return;
    }

    ACC[0] = 8;
    ACC[1] = 8;
    Speed[0] = 200;
    Speed[1] = 200;

    float roll_angle = (float)p_mushibot->mpu6050.getAngleX() + 2.0;
    leg_position_add = pid_roll_angle(lpf_roll(roll_angle)); // test
    Position[0] = 2048 + 12 + 8.4 * (wrobot.height - 32) - leg_position_add;
    Position[1] = 2048 - 12 - 8.4 * (wrobot.height - 32) - leg_position_add;
    
    if (Position[0] < 2110)
        Position[0] = 2110;
    else if (Position[0] > 2510)
        Position[0] = 2510;

    if (Position[1] < 1586)
        Position[1] = 1586;
    else if (Position[1] > 1986)
        Position[1] = 1986;
    
    p_mushibot->sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);    
}

void MotionController::loop_jumping()
{
    if ((wrobot.dir_last == 5) && (wrobot.dir == 4) && (jump_flag == 0))
    {
        ACC[0] = 0;
        ACC[1] = 0;
        Speed[0] = 0;
        Speed[1] = 0;
        Position[0] = 2048 + 12 + 8.4 * (80 - 32);
        Position[1] = 2048 - 12 - 8.4 * (80 - 32);
        p_mushibot->sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

        jump_flag = 1;
    }
    if (jump_flag > 0)
    {
        jump_flag++;
        if ((jump_flag > 30) && (jump_flag < 35))
        {
            ACC[0] = 0;
            ACC[1] = 0;
            Speed[0] = 0;
            Speed[1] = 0;
            Position[0] = 2048 + 12 + 8.4 * (40 - 32);
            Position[1] = 2048 - 12 - 8.4 * (40 - 32);
            p_mushibot->sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

            jump_flag = 40;
        }
        if (jump_flag > 200)
        {
            jump_flag = 0; // 已准备好再次跳跃
        }
    }    
}

void MotionController::loop_joy()
{
    // 倒地失控后关闭输出
    if (abs(LQR_angle) > 25.0f)
    {
        uncontrolable = 1;
    }
    if (uncontrolable != 0) // 扶起后延时恢复
    {
        if (abs(LQR_angle) < 10.0f)
        {
            uncontrolable++;
        }
        if (uncontrolable > 200) // 200次程序循环的延时时间
        {
            uncontrolable = 0;
        }
    }

    // 关停输出（遥控停止或角度过大失控）
    if (wrobot.go == 0 || uncontrolable != 0)
    {
        p_mushibot->motor1.target = 0;
        p_mushibot->motor2.target = 0;
        leg_position_add = 0;
    }

    // 记录上一次的遥控数据数据
    wrobot.dir_last  = wrobot.dir;
    wrobot.joyx_last = wrobot.joyx;
    wrobot.joyy_last = wrobot.joyy;
}

void MotionController::adjust_p_by_height()
{
    // 平衡控制参数自适应
    if (wrobot.height < 50)
    {
        pid_speed.P = 0.7;
    }
    else if (wrobot.height < 64)
    {
        pid_speed.P = 0.6;
    }
    else
    {
        pid_speed.P = 0.5;
    }
}

void MotionController::yaw_angle_addup()
{
    YAW_angle = (float)p_mushibot->mpu6050.getAngleZ();
    YAW_gyro  = (float)p_mushibot->mpu6050.getGyroZ();

    if (YAW_angle_zero_point == (-10))
    {
        YAW_angle_zero_point = YAW_angle;
    }

    float yaw_angle_1, yaw_angle_2, yaw_addup_angle;
    if (YAW_angle > YAW_angle_last)
    {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last - 2 * PI;
    }
    else
    {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last + 2 * PI;
    }

    if (abs(yaw_angle_1) > abs(yaw_angle_2))
    {
        yaw_addup_angle = yaw_angle_2;
    }
    else
    {
        yaw_addup_angle = yaw_angle_1;
    }

    YAW_angle_total = YAW_angle_total + yaw_addup_angle;
    YAW_angle_last = YAW_angle;    
}

int MotionController::is_turn() const
{
    return wrobot.joyx || wrobot.joyy;
}

int MotionController::is_jump() const
{
    return jump_flag;
}

void MotionController::get_pid_params_map(std::map<String, String>& params_map, const PIDController& pid, const String& name) const
{
    params_map.insert(std::make_pair(name + ".P", String(pid.P)));
    params_map.insert(std::make_pair(name + ".I", String(pid.I)));
    params_map.insert(std::make_pair(name + ".D", String(pid.D)));
}

void MotionController::get_params_map(std::map<String, String> &params_map) const
{
    params_map.insert(std::make_pair("robot_speed", String(p_mushibot->get_speed())));
    params_map.insert(std::make_pair("robot_speed_last", String(p_mushibot->get_last_speed())));
    params_map.insert(std::make_pair("motor1_shaft_velocity", String(p_mushibot->motor1.shaft_velocity)));
    params_map.insert(std::make_pair("motor2_shaft_velocity", String(p_mushibot->motor2.shaft_velocity)));

#if 0
    //PID部分
    get_pid_params_map(params_map, pid_angle, "pid_angle");
    get_pid_params_map(params_map, pid_gyro, "pid_gyro");
    get_pid_params_map(params_map, pid_distance, "pid_distance");
    get_pid_params_map(params_map, pid_speed, "pid_speed");
    get_pid_params_map(params_map, pid_yaw_angle, "pid_yaw_angle");
    get_pid_params_map(params_map, pid_yaw_gyro, "pid_yaw_gyro");
    get_pid_params_map(params_map, pid_lqr_u, "pid_lqr_u");
    get_pid_params_map(params_map, pid_zeropoint, "pid_zeropoint");
    get_pid_params_map(params_map, pid_roll_angle, "pid_roll_angle");
#endif

    params_map.insert(std::make_pair("LQR_angle", String(LQR_angle)));
    params_map.insert(std::make_pair("LQR_gyro", String(LQR_gyro)));
    params_map.insert(std::make_pair("LQR_speed", String(LQR_speed)));
    params_map.insert(std::make_pair("LQR_distance", String(LQR_distance)));
    params_map.insert(std::make_pair("angle_control", String(angle_control)));
    params_map.insert(std::make_pair("gyro_control", String(gyro_control)));
    params_map.insert(std::make_pair("speed_control", String(speed_control)));
    params_map.insert(std::make_pair("distance_control", String(distance_control)));
    params_map.insert(std::make_pair("LQR_u", String(LQR_u)));
    params_map.insert(std::make_pair("angle_zeropoint", String(angle_zeropoint)));
    params_map.insert(std::make_pair("distance_zeropoint", String(distance_zeropoint)));
    params_map.insert(std::make_pair("YAW_gyro", String(YAW_gyro)));
    params_map.insert(std::make_pair("YAW_angle", String(YAW_angle)));
    params_map.insert(std::make_pair("YAW_angle_last", String(YAW_angle_last)));
    params_map.insert(std::make_pair("YAW_angle_total", String(YAW_angle_total)));
    params_map.insert(std::make_pair("YAW_angle_zero_point", String(YAW_angle_zero_point)));
    params_map.insert(std::make_pair("YAW_output", String(YAW_output)));
    params_map.insert(std::make_pair("leg_position_add", String(leg_position_add)));

#if 0
    for (uint8_t i = 0; i < 2; i++)
    {
        params_map.insert(std::make_pair("Servo_ID_" + i, String(ID[i])));
        params_map.insert(std::make_pair("Servo_Position_" + i, String(Position[i])));
        params_map.insert(std::make_pair("Servo_Speed_" + i, String(Speed[i])));
        params_map.insert(std::make_pair("Servo_ACC_" + i, String(ACC[i])));
    }
#endif
}