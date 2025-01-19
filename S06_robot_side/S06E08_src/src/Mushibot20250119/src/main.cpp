// Put all the functions that need global variable here.

#include <ArduinoLog.h>

#include "upper_tier/teleop_protocol.h"
#include "upper_tier/telecomm_channel.h"
#include "robot_commander.h"


RobotCommander commander = RobotCommander();
TelecommChannel telecomm = TelecommChannel();
TeleopProtocol protocol = TeleopProtocol();


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
                Serial.printf("\n[EVENT] WebSocket Client '%u' is connected. \n", num);
            }
            break;

        // Echo text message back to client
        case WStype_TEXT:     
            if (length > 0) {
                char payload_chars[length];
                sprintf(payload_chars, "%s", payload);
                String payload_str = String(payload_chars);

                Serial.printf("\n[EVENT] WebSocket Client: '%u', Payload: '%s'\n", num, payload_str.c_str());
                protocol.parse_command(payload_str);
            }

            /*
            {
                JsonDocument status_json = mushibot.get_status();
                String status_str;
                serializeJson(status_json, status_str);
                status_json.clear();
                Serial.printf("\n[DEBUG] handle_WebsocketEvents() status_str: '%s'\n", status_str.c_str());
                telecomm.websocket.broadcastTXT(status_str.c_str());
            }            
            */            

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


void setup()
{
    Serial.begin(115200);   // Log prints here.
    Log.begin(LOG_LEVEL_VERBOSE, &Serial, true);

    telecomm.setup_telecomm();
    commander.setup_command();
 
}

void loop()
{
    // mushibot.motor1.target = 10.0;
    // mushibot.motor2.target = 10.0;

    telecomm.loop_telecomm();
    commander.loop_command();
}

