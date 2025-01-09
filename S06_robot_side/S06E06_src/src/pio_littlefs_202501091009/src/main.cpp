#include "embedded_fs.h"

EmbeddedFS embedded_fs = EmbeddedFS();


void setup()
{
    Serial.begin(115200);   // 通讯串口

    embedded_fs.setup_embeddedfs();
}

void loop()
{
    embedded_fs.loop_embeddedfs();
}

