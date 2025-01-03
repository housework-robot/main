#include "webserver_controller.h"

WebServerController::WebServerController() : robot_protocol(20)
{

}

WebServerController::~WebServerController()
{

}

void WebServerController::setup()
{
    webserver.begin();
    webserver.on("/", HTTP_GET, std::bind(&WebServerController::basicWebCallback, this));
}

void WebServerController::basicWebCallback()
{
    webserver.send(300, "text/html", basic_web);
}

void WebServerController::set_self_banlance()
{
    // 自平衡初始化参数
    String params = "{ \"roll\":0,\"height\":38,\"linear\":0,\"angular\":0,\"stable\":1,\"mode\":\"basic\",\"dir\":\"stop\",\"joy_y\":0,\"joy_x\":0 }";
    JsonDocument doc;
    deserializeJson(doc, params);
    robot_protocol.parseBasic(doc);
    doc.clear();
}

void WebServerController::loop_web()
{
    webserver.handleClient();
    robot_protocol.spinOnce(); // 更新web端回传的控制信息
}

bool WebServerController::parse_basic(const uint8_t *payload)
{
    bool flag = false;
    String payload_str = String((char *)payload);
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload_str);
    if ("basic" == doc["mode"])
    {
        robot_protocol.parseBasic(doc);
        flag = true;
    }
    doc.clear();
    return flag;
}