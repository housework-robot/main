#include "wswifi.h"

// For testing purpose
extern void handle_WebsocketEvents(
    uint8_t num,
    WStype_t type,
    uint8_t * payload,
    size_t length);


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

    // Get the public IP address
    // IPAddress public_ip_addr = get_public_IP_address();
    // Serial.printf("\n[INFO] The Mushibot's public IP address is: '%s' \n", public_ip_addr.toString().c_str());

    
    // Setup the websocket. 
    websocket.begin();
    // websocket.onEvent(webSocketEventCallback);
    websocket.onEvent(handle_WebsocketEvents);
    Serial.printf("\n[INFO] WebSocketsServer is ready.\n");

}


void WsWifi::loop_wswifi() {
    websocket.loop();

    // https get test.
    String website_url_kimi = "https://kimi.moonshot.cn/"; 
    String certificate_filename_kimi = "/cert_store/kimi.moonshot.cn";    
    String website_url_howsmyssl = "https://www.howsmyssl.com/a/check"; 
    String certificate_filename_howsmyssl = "/cert_store/howsmyssl.com";  

    // jiasaw is expected to fail, because it is blocked by GFW.
    String website_url_jiasaw = "https://jigsaw.w3.org/HTTP/connection.html"; 
    String certificate_filename_jiasaw = "/cert_store/jigsaw.w3.org";    

    String https_content = https_get(website_url_kimi, certificate_filename_kimi);  
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
                i, String(WiFi.SSID(i)).c_str(), abs(WiFi.RSSI(i))
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
        Serial.printf("\n[INFO] connect_wifi(): The mushibot is connected to '%s' wifi network, with '%s' local IP address.\n", 
            String(WiFi.SSID()), WiFi.localIP().toString());
    }
}


IPAddress WsWifi::get_public_IP_address() {
    HTTPClient http_client;
    String public_ip_str;
    const char* public_ip_chars;
    JsonDocument public_ip_json;
    IPAddress public_ip_address;
    int httpResponseCode;

    if (WiFi.status() == WL_CONNECTED) {
        http_client.begin(public_ip_api);
        httpResponseCode = http_client.GET();

        if (httpResponseCode > 0) {            
            // public_ip_str = http_client.getString();
            deserializeJson(public_ip_json, http_client.getStream());

            serializeJson(public_ip_json, public_ip_str);
            Serial.printf("\n[DEBUG] The Mushibot's public IP address is: '%s' \n", public_ip_str.c_str());

            public_ip_chars = public_ip_json["ip"];
            // Serial.printf("public_ip_chars: '%s' \n", public_ip_chars);
            public_ip_address.fromString(public_ip_chars);
            // Serial.println(public_ip_address);
        }
        else {
            Serial.printf("\n[WARN] Cannot access '%s' to get public IP address, http code is: '%d'.\n", 
                public_ip_api, httpResponseCode);
        }

        http_client.end();
    }

    return public_ip_address;
}



String WsWifi::https_get(String https_url, String cert_filename) {  
    String website_content;
    String certificate_content; 

    // Get the certificate
    certificate_content = embedded_fs.read_file(cert_filename);

    // To carry https certificate
    NetworkClientSecure *secure_client = new NetworkClientSecure;

    if (secure_client) {
        secure_client->setCACert(certificate_content.c_str());

        // Connect the https website
        if (https_client.begin(*secure_client, https_url)) { 
            int httpCode = https_client.GET();        

            if (httpCode > 0) {
                // HTTP header has been send and Server response header has been handled
                Serial.printf("\n[DEBUG] GET returns code: %d\n", httpCode);
        
                // file found at server
                if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
                    // print server response payload
                    website_content = https_client.getString();
                    Serial.printf("\n[DEBUG] The following is the content retrieved from '%s'\n", https_url.c_str());
                    Serial.println(website_content.c_str());
                    Serial.printf("\n[DEBUG] The above content is retrieved from '%s'\n", https_url.c_str());
                }
            }
            else {
                Serial.printf("[WARN] GET '%s' failed, error: %s\n", 
                    https_url.c_str(),
                    https_client.errorToString(httpCode).c_str()
                );
            }

            https_client.end();
        }

        delete secure_client;
    }
    else {
        Serial.printf("\n[WARN] Unable to create NetworkClientSecure instance.\n");
    }

    return website_content;
}
