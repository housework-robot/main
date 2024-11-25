#include "balancing_bot.h"

/*
BalancingBot::BalancingBot() {
  Serial.printf("Start up the balancing bot.\n");
}
*/

void BalancingBot::I2C_init(){
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  // Setup position sensors.
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);
}

void BalancingBot::setup() {

  I2C_init();

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // link the motor to the sensor
  motor0.linkSensor(&sensor0);
  motor1.linkSensor(&sensor1);

  // Velocity-oriented PID parameters.
  motor1.PID_velocity.P = 0.01;
  motor1.PID_velocity.I = 0.1;
  motor1.PID_velocity.D = 0;

  motor0.PID_velocity.P = 0.01;
  motor0.PID_velocity.I = 0.1;
  motor0.PID_velocity.D = 0;

  motor0.voltage_sensor_align = 2;
  driver0.voltage_power_supply = 12;
  driver0.init();
  motor0.linkDriver(&driver0);

  motor1.voltage_sensor_align = 2;
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);

  motor0.torque_controller = TorqueControlType::voltage;
  motor1.torque_controller = TorqueControlType::voltage;
  motor0.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::torque;

  // enable monitoring
  motor1.useMonitoring(Serial);
  motor0.useMonitoring(Serial);

  // initialise motor
  motor1.init();
  motor0.init();

  // align encoder and start FOC
  motor1.initFOC();
  motor0.initFOC();
}

JsonDocument BalancingBot::get_observation() {
  JsonDocument obs;

  mpu6050.update();
  obs["pitch_angle"] = mpu6050.getAngleY();
  obs["motor0_velocity"] = motor0.shaft_velocity;
  obs["motor1_velocity"] = motor1.shaft_velocity;
  return obs;
}

JsonDocument BalancingBot::policy(
    JsonDocument observation, 
    JsonDocument command
  ) {

  JsonDocument action;

  float cmd_throttle = lpf_throttle(command["throttle"]);
  float cmd_steering = lpf_steering(command["steering"]);

  float motor0_velocity = float(observation["motor0_velocity"]);
  float motor1_velocity = float(observation["motor1_velocity"]);
  float target_pitch = lpf_pitch_cmd(
    pid_vel((M0 * motor0_velocity + M1 * motor1_velocity) / 2 - cmd_throttle)
  );

  float pitch_angle = float(observation["pitch_angle"]);
  float voltage_control = pid_stb(Offset_parameters - pitch_angle + target_pitch);

  action["motor0_target"] = M0 * voltage_control - cmd_steering;
  action["motor1_target"] = M1 * voltage_control + cmd_steering;
  return action;
}

void BalancingBot::step(
    JsonDocument action
  ) {

  motor0.target = action["motor0_target"];
  motor1.target = action["motor1_target"];

  motor0.loopFOC();
  motor1.loopFOC();
  motor0.move();
  motor1.move(); 
}
