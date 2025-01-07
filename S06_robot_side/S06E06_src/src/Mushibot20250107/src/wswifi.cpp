#include "wswifi.h"

extern void webSocketEventCallback(
    uint8_t num, 
    WStype_t type, 
    uint8_t *payload, 
    size_t length
);


WsWifi::WsWifi() {
}

WsWifi::~WsWifi() {
    WiFi.disconnect();
}

void handle_wifiConnected(WiFiEvent_t event, WiFiEventInfo_t info){
    Serial.printf("\n[EVENT] The mushibot is connected to wifi network '%s', waiting for getting IP address.\n", 
        String(WiFi.SSID()) );
}

void handle_wifiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
    Serial.printf("\n[EVENT] The mushibot is connected to '%s', with '%s' IP address.\n", 
        String(WiFi.SSID()), WiFi.localIP().toString());
}

void handle_wifiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
    Serial.printf("\n[EVENT] The mushibot is disconnected from wifi network.\n" );
    // Serial.printf("\t Reason: %s \n", info.wifi_sta_disconnected.reason);
}

void WsWifi::setup_wswifi() {
    // Setup the LittleFS.
    embedded_fs.setup_embeddedfs();

    // Setup the wifi. 
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(1000);

    WiFi.onEvent(handle_wifiConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(handle_wifiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(handle_wifiDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    scan_wifi();
    connect_wifi(sta_ssid, sta_password); 
    
    // Setup the websocket. 
    websocket.begin();
    websocket.onEvent(webSocketEventCallback);
}


void WsWifi::loop_wswifi() {
    embedded_fs.loop_embeddedfs();
    websocket.loop();
}


void WsWifi::scan_wifi() {
    String wifi_auth = "";
    Serial.printf("\n[INFO] Scan wifi start:\n");

    // WiFi.scanNetworks will return the number of networks found
    int num_wifi = WiFi.scanNetworks();
    delay(1000);

    if (num_wifi == 0) {
        Serial.printf("\t No wifi network is found.\n");
    } 
    else {  
        for (int i = 0; i < num_wifi; ++i) {
            // wifi_auth = String((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " without password" : " with password");
            Serial.printf("   [%d] Wifi-name: '%s', Signal-strength: %d, Authentication: ",
                i, String(WiFi.SSID(i)), abs(WiFi.RSSI(i))
            );
            Serial.println(
                (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " without password." : " with password."
            );
        }
    }
    Serial.println("");
}

void WsWifi::connect_wifi(String ssid, String password)
{
    int cnt = 0;
    WiFi.begin(ssid, password);
    delay(2000);

    // 等待连接
    while (WiFi.status()!=WL_CONNECTED) { 
        WiFi.begin(ssid, password);

        cnt += 1;
        if (cnt > 5) {
            break;
        }
        delay(2000);
    }

    if (cnt > 5) {
        Serial.printf("\n[WARN] connect_wifi(): The mushibot can NOT connected to '%s' wifi network.\n", 
            String(WiFi.SSID()));
    } 
    else {
        robot_ip = WiFi.localIP();
        Serial.printf("\n[INFO] connect_wifi(): The mushibot is connected to '%s' wifi network, with '%s' IP address.\n", 
            String(WiFi.SSID()), WiFi.localIP().toString());
    }
}
