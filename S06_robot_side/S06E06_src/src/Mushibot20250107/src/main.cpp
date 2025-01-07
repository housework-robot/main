// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------

// 机器人控制头文件

#include "mushibot.h"
#include "wswifi.h"
#include "motion_controller.h"

Mushibot mushibot = Mushibot();
WsWifi wswifi = WsWifi();
MotionController motion_controller = MotionController();

#define S_RXD 16 // Setting the serial receive pin
#define S_TXD 17 // Setting the serial transmit pin


String json2string(JsonDocument& doc)
{
    String status_string;
    // 将JSON文档转换为字符串
    serializeJson(doc, status_string);
    doc.clear();

    return status_string;
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    if (type == WStype_CONNECTED)
    {
        printf("[EVENT] %s connected to %s\n", wswifi.robot_ip, wswifi.sta_ssid);
    }
    if (type == WStype_TEXT)
    {
        JsonDocument status_json = mushibot.get_status();
        String status_str = json2string(status_json);
        wswifi.websocket.broadcastTXT(status_str);
    }
}

void setup()
{
    Serial.begin(115200);   // 通讯串口

    // The configuration of the serial channel may specify the number of data bits, 
    // stop bits, parity, flow control and other setting. 
    // The default is SERIAL_8N1, 8 data bits, no parity, and 1 stop bit. 
    Serial2.begin(1000000, SERIAL_8N1, S_RXD, S_TXD); // 腿部sts舵机
    
    wswifi.setup_wswifi();

    mushibot.setup_mushibot();
    motion_controller.p_mushibot = &mushibot;
    
    // web_controller.setup();    
}

void loop()
{
    // mushibot.motor1.target = 10.0;
    // mushibot.motor2.target = 10.0;

    mushibot.loop_mushibot();
    
    /*
    float LQR_u = motion_controller.loop_pitch();

    float weight = 0.1;
    mushibot.motor1.target = (-weight) * LQR_u;
    mushibot.motor2.target = (-weight) * LQR_u;

    Serial.printf("LQR_u=%s \n", LQR_u);
    // Serial.println(LQR_u);

    wswifi.loop_wswifi();
    */
}

