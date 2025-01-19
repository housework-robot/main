#include "telecomm_channel.h"

extern void handle_WebsocketEvents(
    uint8_t num,
    WStype_t type,
    uint8_t * payload,
    size_t length
);

extern void handle_wifiConnected(
    WiFiEvent_t event, 
    WiFiEventInfo_t info
);

extern void handle_wifiGotIP(
    WiFiEvent_t event, 
    WiFiEventInfo_t info
);

extern void handle_wifiDisconnected(
    WiFiEvent_t event, 
    WiFiEventInfo_t info
);




TelecommChannel::TelecommChannel() {
}

TelecommChannel::~TelecommChannel() {
    WiFi.disconnect();
}



void TelecommChannel::setup_telecomm() {
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
    String local_ip = connect_wifi(sta_ssid, sta_password); 

    // Get the public IP address
    // IPAddress public_ip_addr = get_public_IP_address();
    // Serial.printf("\n[INFO] The Mushibot's public IP address is: '%s' \n", public_ip_addr.toString().c_str());

    
    // Setup the websocket. 
    websocket.begin();
    // websocket.onEvent(webSocketEventCallback);
    websocket.onEvent(handle_WebsocketEvents);
    Serial.printf("\n[INFO] WebSocketsServer is ready.\n");

    bool http_available = is_http_available();
    if (http_available) { 
        JsonDocument json_doc;
        json_doc["robot_ip"] = local_ip;
        String http_post_json = http_post(http_website_url, json_doc);
    }
}


bool TelecommChannel::is_http_available() {
    // https get test.
    String website_url_kimi = "https://kimi.moonshot.cn/"; 
    String certificate_filename_kimi = "/cert_store/kimi.moonshot.cn";    
    String website_url_howsmyssl = "https://www.howsmyssl.com/a/check"; 
    String certificate_filename_howsmyssl = "/cert_store/howsmyssl.com";  

    // jiasaw is expected to fail, because it is blocked by GFW.
    String website_url_jiasaw = "https://jigsaw.w3.org/HTTP/connection.html"; 
    String certificate_filename_jiasaw = "/cert_store/jigsaw.w3.org";    

    // String https_get_content = https_get(website_url_kimi, certificate_filename_kimi);  

    // self-constructed expressjs website  
    String http_get_express = http_get(http_website_url + "/hello/get_ExpressStyle");
    String http_get_query = http_get(http_website_url + "/hello?name=get_HttpQueryStyle");
    String http_post_str = http_post(http_website_url, "{\"name\": \"post in String.\"}");

    JsonDocument json_doc;
    json_doc["robot_ip"] = local_ip;
    String http_post_json = http_post(http_website_url, json_doc);

    if (http_post_json.length() > 0) {
        return true;
    }
    else {
        return false;
    }
}


void TelecommChannel::loop_telecomm() {
    websocket.loop();
}


void TelecommChannel::scan_wifi() {
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


String TelecommChannel::connect_wifi(String ssid, String password)
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
        local_ip = WiFi.localIP().toString();
        Serial.printf("\n[INFO] connect_wifi(): The mushibot is connected to '%s' wifi network, with '%s' local IP address.\n", 
            String(WiFi.SSID()), local_ip);    
    }

    return local_ip;
}


IPAddress TelecommChannel::get_public_IP_address() {
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
            public_ip = String(public_ip_chars);

            public_ip_address.fromString(public_ip_chars);
        }
        else {
            Serial.printf("\n[WARN] Cannot access '%s' to get public IP address, http code is: '%d'.\n", 
                public_ip_api, httpResponseCode);
        }

        http_client.end();
    }

    return public_ip_address;
}


String TelecommChannel::http_post(String http_url, String payload_str) {
    HTTPClient http_client;
    int http_res_code;
    String http_res_str;

    if (WiFi.status() == WL_CONNECTED) {
        http_client.begin(http_url + "/post_text");
        http_client.addHeader("Content-Type", "application/json");

        String serialized_json; 
        JsonDocument payload_json;
        payload_json["datatype"] = "text";
        payload_json["content"] = payload_str;
        serializeJson(payload_json, serialized_json);

        http_res_code = http_client.POST(serialized_json);

        if (http_res_code == HTTP_CODE_OK) {            
            http_res_str = http_client.getString();
            Serial.printf("\n[DEBUG] The following content is retrieved from '%s' by 'HTTP POST' \n", http_url.c_str());
            Serial.println(http_res_str.c_str());
            Serial.printf("[DEBUG] The above content is retrieved from '%s' by 'HTTP POST'\n", http_url.c_str());
        }
        else {
            Serial.printf("\n[WARN] Cannot access '%s' for http POST, http code is: '%d'.\n", 
                http_url.c_str(), http_res_code);
        }

        http_client.end();
    }

    return http_res_str;    
}



String TelecommChannel::http_post(String http_url, JsonDocument payload_json) {
    HTTPClient http_client;
    int http_res_code;
    String http_res_str;

    if (WiFi.status() == WL_CONNECTED) {
        http_client.begin(http_url + "/post_json");
        http_client.addHeader("Content-Type", "application/json");

        String payload_str; 
        payload_json["datatype"] = "json";
        serializeJson(payload_json, payload_str);

        http_res_code = http_client.POST(payload_str);

        if (http_res_code == HTTP_CODE_OK) {            
            http_res_str = http_client.getString();
            Serial.printf("\n[DEBUG] The following content is retrieved from '%s' by 'HTTP POST' \n", http_url.c_str());
            Serial.println(http_res_str.c_str());
            Serial.printf("[DEBUG] The above content is retrieved from '%s' by 'HTTP POST'\n", http_url.c_str());
        }
        else {
            Serial.printf("\n[WARN] Cannot access '%s' for http POST, http code is: '%d'.\n", 
                http_url.c_str(), http_res_code);
        }

        http_client.end();
    }

    return http_res_str;
}


String TelecommChannel::http_get(String http_url) {
    HTTPClient http_client;
    int http_res_code;
    String http_res_str;

    if (WiFi.status() == WL_CONNECTED) {
        http_client.begin(http_url);
        http_res_code = http_client.GET();

        if (http_res_code == HTTP_CODE_OK) {            
            http_res_str = http_client.getString();
            Serial.printf("\n[DEBUG] The following content is retrieved from '%s' by 'HTTP GET' \n", http_url.c_str());
            Serial.println(http_res_str.c_str());
            Serial.printf("[DEBUG] The above content is retrieved from '%s' by 'HTTP GET'\n", http_url.c_str());
        }
        else {
            Serial.printf("\n[WARN] Cannot access '%s' for http GET, http code is: '%d'.\n", 
                http_url.c_str(), http_res_code);
        }

        http_client.end();
    }

    return http_res_str;
}


String TelecommChannel::https_get(String https_url, String cert_filename) {  
    String https_res_str;
    String certificate_content; 

    // Get the certificate
    certificate_content = embedded_fs.read_file(cert_filename);

    // To carry https certificate
    NetworkClientSecure *secure_client = new NetworkClientSecure;

    if (secure_client) {
        secure_client->setCACert(certificate_content.c_str());

        // Connect the https website
        if (http_client.begin(*secure_client, https_url)) { 
            int httpCode = http_client.GET();        

            if (httpCode > 0) {
                // HTTP header has been send and Server response header has been handled
                Serial.printf("\n[DEBUG] GET returns code: %d\n", httpCode);
        
                // file found at server
                if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
                    // print server response payload
                    https_res_str = http_client.getString();
                    Serial.printf("\n[DEBUG] The following is the content retrieved from '%s' by 'HTTPS GET'\n", https_url.c_str());
                    Serial.println(https_res_str.c_str());
                    Serial.printf("[DEBUG] The above content is retrieved from '%s'  by 'HTTPS GET'\n", https_url.c_str());
                }
            }
            else {
                Serial.printf("[WARN] GET '%s' failed, error: %s\n", 
                    https_url.c_str(),
                    http_client.errorToString(httpCode).c_str()
                );
            }

            http_client.end();
        }

        delete secure_client;
    }
    else {
        Serial.printf("\n[WARN] Unable to create NetworkClientSecure instance.\n");
    }

    return https_res_str;
}