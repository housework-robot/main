#include "teleop_protocol.h"


TeleopProtocol::TeleopProtocol() {
}

TeleopProtocol::~TeleopProtocol() {
}


void TeleopProtocol::parse_command(String &teleop_command) {
    JsonDocument json_doc;
    Serial.printf("\n[DEBUG] parse_command, teleop_command: '%s'\n", teleop_command.c_str());
    deserializeJson(json_doc, teleop_command.c_str());

    // Teleoperation command parameters
    go = true;   
    stable = true;  // Stand stably.
    mode = String(json_doc["mode"]);

    height = int(json_doc["height"]);
    roll = int(json_doc["roll"]);
    angular  = int(json_doc["angular"]); 
    linear = int(json_doc["linear"]); 

    dir_last = dir;
    dir = String(json_doc["dir"]);
    
    joyy_last = joyy;
    joyy = int(json_doc["joy_y"]);
   
    joyx_last = joyx;
    joyx = int(json_doc["joy_x"]);
}