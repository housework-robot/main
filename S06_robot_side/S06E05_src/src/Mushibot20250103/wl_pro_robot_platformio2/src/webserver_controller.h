#ifndef WEBSERVER_CONTROLLER_H_
#define WEBSERVER_CONTROLLER_H_

#include <WebServer.h>
#include "basic_web.h"
#include "robot.h"

class WebServerController
{
public:
    WebServerController();
    virtual ~WebServerController();

    void setup();
    void loop_web();
    bool parse_basic(const uint8_t *payload);
    void set_self_banlance();

private:
    void basicWebCallback();

    WebServer webserver;
    RobotProtocol robot_protocol;
};

#endif