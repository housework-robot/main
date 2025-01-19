#include "robot_commander.h"

extern TeleopProtocol protocol;

RobotCommander::RobotCommander()
{
}

RobotCommander::~RobotCommander()
{
}


void RobotCommander::setup_command() 
{
    mushibot.setup_mushibot();
}

void RobotCommander::loop_command()
{
    if (protocol.dir == "jump") {
        Log.verboseln("dir = 'jump', height = '%d'", protocol.height);
    }
    else if (protocol.dir == "forward") {
        Log.verboseln("dir = 'forward', linear = '%d'", protocol.linear);
    }
    else if (protocol.dir == "back") {
        Log.verboseln("dir = 'back', linear = '%d'", protocol.linear);
    }
    else if (protocol.dir == "left") {
        Log.verboseln("dir = 'left', angular = '%d'", protocol.angular);
    }
    else if (protocol.dir == "right") {
        Log.verboseln("dir = 'right', angular = '%d'", protocol.angular);
    }

    mushibot.loop_mushibot();
}