#ifndef ROBOT_COMMANDER_H_
#define ROBOT_COMMANDER_H_

#include <ArduinoLog.h>

#include "upper_tier/teleop_protocol.h"
#include "lower_tier/mushibot.h"


class RobotCommander
{
public:
    RobotCommander();
    virtual ~RobotCommander();

    void setup_command();
    void loop_command();

    Mushibot mushibot = Mushibot();

private:

    // 逻辑处理标志位
    float robot_speed = 0;         // 记录当前轮部转速
    float robot_speed_last = 0;    // 记录上一时刻的轮部转速
    int jump_flag = 0;             // 跳跃时段标识
    float leg_position_add = 0;    // roll轴平衡控制量
    int uncontrolable = 0;         // 机身倾角过大导致失控

};

#endif


