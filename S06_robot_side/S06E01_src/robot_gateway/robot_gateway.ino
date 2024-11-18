#include <Arduino.h>
#include "balancing_bot.h"

String chipId;
BalancingBot blc_bot;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    delay(750);

    chipId = String((uint32_t)ESP.getEfuseMac(), HEX);
    chipId.toUpperCase();
    Serial.printf("chipId: %d \n", chipId);

    blc_bot.setup();
}

void loop() {
  // put your main code here, to run repeatedly:

  // remote command is received by the upper_tier from the remote server.
  std::map<std::string, float> cmd;
  cmd["throttle"] = 0.0;
  cmd["steering"] = 0.0;

  std::map<std::string, float> obs = blc_bot.get_observation();
  std::map<std::string, float> action = blc_bot.policy(obs, cmd);

  /*
  Serial.printf("pitch_angle: %f, motor0_velocity: %f, motor1_velocity: %f, motor0_target: %f, motor1_target: %f \n", 
    obs["pitch_angle"], obs["motor0_velocity"], obs["motor1_velocity"], action["motor0_target"], action["motor1_target"]
  );
  */
  Serial.printf("%f %f %f %f %f\n", 
    obs["pitch_angle"], obs["motor0_velocity"], obs["motor1_velocity"], action["motor0_target"], action["motor1_target"]
  );
  blc_bot.step(action);
}
