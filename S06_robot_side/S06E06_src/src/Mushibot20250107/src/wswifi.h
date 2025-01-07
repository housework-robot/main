#ifndef WSWIFI_H_
#define WSWIFI_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>

#include <WiFi.h>
#include <WebSocketsServer.h>

#include "embedded_fs.h"


class WsWifi
{
public:
    WsWifi();
    virtual ~WsWifi();

    // LittleFS wrapped as EmbeddedFS
    EmbeddedFS embedded_fs = EmbeddedFS();
    
    // Wifi with station mode
    String sta_ssid = "KanHome";
    String sta_password = "";
    // String sta_ssid = "RHKJ";
    // String sta_password = "rhkj1234";
    IPAddress robot_ip;

    void setup_wswifi(); 
    void loop_wswifi();

    void scan_wifi();
    void connect_wifi(String ssid, String password);    

    // WebSocketsServer with port = 81
    WebSocketsServer websocket = WebSocketsServer(81);

private:   
};

#endif