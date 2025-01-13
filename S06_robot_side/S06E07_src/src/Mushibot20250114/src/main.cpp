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


void handle_WebsocketEvents(
    uint8_t num,
    WStype_t type,
    uint8_t * payload,
    size_t length) 
{
    // Figure out the type of WebSocket event
    switch (type) {

        // Client has disconnected
        case WStype_DISCONNECTED:
            Serial.printf("\n[EVENT] WebSocket Client '%u' is disconnected. \n", num);
            break;

        // New client has connected
        case WStype_CONNECTED:    
            {
                IPAddress ip = wswifi.websocket.remoteIP(num);
                Serial.printf("\n[EVENT] WebSocket Client '%u' is connected from ", num);
                Serial.println(ip.toString());
            }
            break;

        // Echo text message back to client
        case WStype_TEXT:
            Serial.printf("\n[EVENT] WS-WebSocket Client: '%u', Payload: '%s'\n", num, payload);

            // For testing purpose. 
            // wswifi.websocket.sendTXT(num, payload);  

            {
                JsonDocument status_json = mushibot.get_status();
                String status_str;
                serializeJson(status_json, status_str);
                status_json.clear();
                Serial.printf("\n[DEBUG] handle_WebsocketEvents() status_str: '%s'\n", status_str.c_str());
                wswifi.websocket.broadcastTXT(status_str.c_str());
            }
            break;

        // For everything else: do nothing
        case WStype_BIN:
        case WStype_ERROR:
            Serial.printf("\n[EVENT] WS-WebSocket Client: '%u', WStype_ERROR, Payload: '%s'\n", num, payload);
            // wswifi.websocket.sendTXT(num, payload);
            break;

        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
        default:
        break;
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
    wswifi.loop_wswifi();
    
    /*
    float LQR_u = motion_controller.loop_pitch();

    float weight = 0.1;
    mushibot.motor1.target = (-weight) * LQR_u;
    mushibot.motor2.target = (-weight) * LQR_u;

    Serial.printf("LQR_u=%s \n", LQR_u);
    // Serial.println(LQR_u);
    */
}

