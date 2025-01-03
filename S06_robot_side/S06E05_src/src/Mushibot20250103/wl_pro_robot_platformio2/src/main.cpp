#include "mushibot.h"
#include "wswifi.h"
#include "motion_controller.h"
#include "webserver_controller.h"

Mushibot mushibot;
WsWifi wswifi;
MotionController motion_controller;
WebServerController web_controller;

static const uint16_t MAX_LOOP_COUNT = 3000;
uint16_t gs_count = 0;

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    if (type == WStype_CONNECTED)
    {
        printf("%s connected to %s\n", wswifi.robot_ip, wswifi.sta_ssid);
    }

    if (type == WStype_TEXT)
    {
        if (false == web_controller.parse_basic(payload))
        {
            std::map<String, String> paramMap;
            motion_controller.get_params_map(paramMap);
            
            String status_str;
            mushibot.get_status(status_str, paramMap);
            wswifi.websocket.broadcastTXT(status_str);
        }
    }
}

void setup()
{
    Serial.begin(115200);   // 通讯串口
    Serial2.begin(1000000); // 腿部sts舵机
    
    wswifi.setup_wswifi();

    mushibot.setup_mushibot();
    motion_controller.p_mushibot = &mushibot;
    
    web_controller.setup();
}

void loop()
{
    if(gs_count < MAX_LOOP_COUNT)
    {
        gs_count++;
    }
    else if(MAX_LOOP_COUNT == gs_count)
    {
        web_controller.set_self_banlance();
        gs_count++;
    }

    web_controller.loop_web();
    wswifi.loop_wswifi();

    mushibot.loop_mushibot(); 

#if 1
    float speed = abs(mushibot.get_speed());

    if (motion_controller.is_jump())
    {
        motion_controller.loop_jump();  //跳跃状态
    }
    else if (motion_controller.is_turn())
    {
        motion_controller.loop_turn();  //转弯状态
    }
    else if (speed > 15)
    {
        motion_controller.loop_motion(); //直线运动状态
    }
    else
    {
        motion_controller.loop_peace(); //平衡状态
    }
#else
    motion_controller.loop();
#endif

    //迭代计算FOC相电压
    mushibot.motor1.loopFOC();
    mushibot.motor2.loopFOC();

    //设置轮部电机输出
    mushibot.motor1.move();
    mushibot.motor2.move();
}