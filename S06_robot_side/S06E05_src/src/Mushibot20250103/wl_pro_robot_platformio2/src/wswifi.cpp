#include "wswifi.h"

extern void webSocketEventCallback(
    uint8_t num, 
    WStype_t type, 
    uint8_t *payload, 
    size_t length);

WsWifi::WsWifi()
{
    // sta_ssid = "robot";
    // sta_password = "Robot@123";

    sta_ssid = "RHKJ";
    sta_password = "rhkj1234";
}

WsWifi::~WsWifi()
{
    
}

void WsWifi::setup_wswifi()
{
    connect_wifi(sta_ssid, sta_password); 
    
    websocket.begin();
    websocket.onEvent(webSocketEventCallback);
}

void WsWifi::loop_wswifi()
{
    websocket.loop();
}

void WsWifi::connect_wifi(const String& ssid, const String& password)
{
  WiFi.begin(ssid, password);

  // 等待连接
  while(WiFi.status()!=WL_CONNECTED)
  {
    //Serial.print(".");
    delay(500);
  }

  robot_ip = WiFi.localIP();

  // 打印ESP-01S的IP地址
  Serial.println("");
  Serial.print("Robot IP Address: ");
  Serial.println(robot_ip);
}