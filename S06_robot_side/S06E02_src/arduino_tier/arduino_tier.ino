// robot_gateway.ino
#include <Arduino.h>
#include <SerialTransfer.h>
#include "balancing_bot.h"

String chipId;
SerialTransfer serial_channel;
BalancingBot blc_bot;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    serial_channel.begin(Serial);
    delay(750);

    chipId = String((uint32_t)ESP.getEfuseMac(), HEX);
    chipId.toUpperCase();
    Serial.printf("chipId: %d \n", chipId);

    blc_bot.setup();
}


// sendJson should not be larger than 1024 characters. 
void send_json(JsonDocument sendJson) {
  uint16_t sendSize = 0;
  char sendStr[1024]; 

  serializeJson(sendJson, sendStr);
  sendSize = serial_channel.txObj(sendStr, sendSize, strlen(sendStr));

  serial_channel.sendData(sendSize);
  // delay(500);
  // serial_channel.reset();
}


JsonDocument receive_json() {
  uint16_t recvSize = 0;
  char recvStr[1024]; 
  JsonDocument recvJson;

  if (serial_channel.available()) {
    recvSize = serial_channel.rxObj(recvStr, recvSize);
    // serial_channel.reset();
    
    Serial.printf("recStr: '%s'\n", recvStr);
    deserializeJson(recvJson, recvStr);
  }  
  else {
    recvJson["throttle"] = 0.0;
    recvJson["steering"] = 0.0;    
  }

  return recvJson;python3 serial_channel.py
}


void loop() {
  // put your main code here, to run repeatedly:

  // remote command is received by the upper_tier from the remote server.
  JsonDocument cmd = receive_json();
  send_json(cmd);

  JsonDocument obs = blc_bot.get_observation();
  send_json(obs);
  JsonDocument action = blc_bot.policy(obs, cmd);

  /*
  Serial.printf("pitch_angle: %f, motor0_velocity: %f, motor1_velocity: %f, motor0_target: %f, motor1_target: %f \n", 
    obs["pitch_angle"], obs["motor0_velocity"], obs["motor1_velocity"], action["motor0_target"], action["motor1_target"]
  );
  */
  Serial.printf("%f %f %f %f %f\n", 
    obs["pitch_angle"], obs["motor0_velocity"], obs["motor1_velocity"], action["motor0_target"], action["motor1_target"]
  );
  blc_bot.step(action);
  // delay(500);
}
