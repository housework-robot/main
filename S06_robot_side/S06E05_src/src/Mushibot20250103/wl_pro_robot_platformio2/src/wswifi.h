#ifndef WSWIFI_H_
#define WSWIFI_H_

#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <FS.h>

class WsWifi
{
public:
    WsWifi();
    virtual ~WsWifi();

    void setup_wswifi(); 
    void loop_wswifi();
    static String json2string(JsonDocument& doc);

    String sta_ssid;
    String sta_password;

    // Port = 81
    WebSocketsServer websocket = WebSocketsServer(81);
    IPAddress robot_ip;

private:
    void connect_wifi(const String& ssid, const String& password);
};

#endif