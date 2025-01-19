#ifndef TELEOP_PROTOCOL_H_
#define TELEOP_PROTOCOL_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoLog.h>


class TeleopProtocol
{
public:
    TeleopProtocol();
    virtual ~TeleopProtocol();

    void parse_command(String &teleop_command);

    // Teleoperation command parameters
    bool go = true;   // What is this?
    bool stable = true;  // What is this?
    String mode = "basic";

    int height = 38;
    int roll = 0;
    int angular  = 0;  // angular velocity
    int linear;  // linear velocity

    String dir = "stop";
    String dir_last = "stop";
    int joyy = 0;
    int joyy_last = 0;
    int joyx = 0;
    int joyx_last = 0;

private:
};


#endif