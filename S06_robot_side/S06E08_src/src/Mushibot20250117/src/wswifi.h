#ifndef WSWIFI_H_
#define WSWIFI_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <vector>

#include <WiFi.h>
#include <HTTPClient.h>
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
    String connect_wifi(String ssid, String password);    

    // Get public IP address
    IPAddress get_public_IP_address();

    // WebSocketsServer
    WebSocketsServer websocket = WebSocketsServer(81);

    // HTTP/S client
    HTTPClient http_client;
    String http_post(String website_url, String payload_str);
    String http_post(String website_url, JsonDocument payload_json);
    String http_get(String website_url);
    String https_get(String website_url, String certificate_filename);

private:   
    // Public IP address service responds in single string. 
    // const char *public_ip_api = "https://realip.cc/simple";   // ends with '\n', error prone.

    // Public IP address service responds in Json.
    const char *public_ip_api = "https://realip.cc/";

    // Verify if the http website is avaiable. 
    bool is_http_available();
};

#endif