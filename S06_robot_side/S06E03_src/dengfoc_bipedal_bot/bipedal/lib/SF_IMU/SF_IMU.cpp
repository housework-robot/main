#include "SF_IMU.h"

SF_IMU::SF_IMU(TwoWire &i2c)
  : _i2c(&i2c) {
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

void SF_IMU::init() {
  writeToIMU(MPU6050_SMPLRT_DIV, 0x00);    //将采样率分频器（SMPLRT_DIV）寄存器设为0x00，即采样率不分频，传感器输出的采样率为陀螺仪的输出速率（一般为8kHz）
  writeToIMU(MPU6050_CONFIG, 0x00);        //将配置（CONFIG）寄存器设为0x00，即关闭低通滤波器，选择内部8kHz时钟
  writeToIMU(MPU6050_GYRO_CONFIG, 0x08);   //将陀螺仪配置（GYRO_CONFIG）寄存器设为0x08，即将陀螺仪量程设置为±500度/秒
  writeToIMU(MPU6050_ACCEL_CONFIG, 0x00);  //将加速度计配置（ACCEL_CONFIG）寄存器设为0x00，即将加速度计量程设置为±2g。
  writeToIMU(MPU6050_PWR_MGMT_1, 0x01);    //将电源管理1（PWR_MGMT_1）寄存器设为0x01，即将传感器置于工作模式，并选择内部时钟源。
  this->update();
  angleGyro[0] = 0;
  angleGyro[1] = 0;
  angleRPY[0] = angleAcc[0];
  angleRPY[1] = angleAcc[1];
  preInterval = millis();
  calGyroOffsets();
}

void SF_IMU::calGyroOffsets() {
  float x = 0, y = 0, z = 0;
  int16_t rx, ry, rz;

  delay(1000);
  Serial.println();
  Serial.println("========================================");
  Serial.println("Calculating gyro offsets");
  Serial.print("DO NOT MOVE MPU6050");

  for (int i = 0; i < 3000; i++) {
    if (i % 1000 == 0) Serial.print(".");

    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(0x43);
    _i2c->endTransmission(false);
    _i2c->requestFrom((int)MPU6050_ADDR, 6);

    rx = _i2c->read() << 8 | _i2c->read();
    ry = _i2c->read() << 8 | _i2c->read();
    rz = _i2c->read() << 8 | _i2c->read();

    x += ((float)rx) / 65.5;
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
  }

  gyroOffset[0] = x / 3000;
  gyroOffset[1] = y / 3000;
  gyroOffset[2] = z / 3000;

  Serial.println("Done!");
  Serial.printf("%f,%f,%f\n", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
  Serial.print("========================================");

  delay(1000);
}


void SF_IMU::update() {
  _i2c->beginTransmission(MPU6050_ADDR);
  _i2c->write(0x3B);
  _i2c->endTransmission(false);
  _i2c->requestFrom((int)MPU6050_ADDR, 14);  //从MPU6050传感器读取加速度计、温度计和陀螺仪的数据，并存储在I²C总线的接收缓冲区中

  //获取寄存器原生数据
  int16_t rawAccX = _i2c->read() << 8 | _i2c->read();
  int16_t rawAccY = _i2c->read() << 8 | _i2c->read();
  int16_t rawAccZ = _i2c->read() << 8 | _i2c->read();
  int16_t rawTemp = _i2c->read() << 8 | _i2c->read();
  int16_t rawGyroX = _i2c->read() << 8 | _i2c->read();
  int16_t rawGyroY = _i2c->read() << 8 | _i2c->read();
  int16_t rawGyroZ = _i2c->read() << 8 | _i2c->read();

  //计算温度
  temp = (rawTemp + 12412.0) / 340.0;

  //计算加速度 16384.0是因为加速度计的量程是±2g，对应的分辨率是16位。
  acc[0] = ((float)rawAccX) / 16384.0;
  acc[1] = ((float)rawAccY) / 16384.0;
  acc[2] = ((float)rawAccZ) / 16384.0;

  //计算加速度角度 使用加速度计数据计算两个轴的倾角
  angleAcc[0] = atan2(acc[1], acc[2] + abs(acc[0])) * 360 / 2.0 / M_PI;
  angleAcc[1] = atan2(acc[0], acc[2] + abs(acc[1])) * 360 / -2.0 / M_PI;

  //计算陀螺仪速度 除以65.5是因为陀螺仪的量程是±500度/秒，对应的分辨率是16位。然后减去陀螺仪的偏移值以消除零点漂移。
  gyro[0] = ((float)rawGyroX) / 65.5 - gyroOffset[0];
  gyro[1] = ((float)rawGyroY) / 65.5 - gyroOffset[1];
  gyro[2] = ((float)rawGyroZ) / 65.5 - gyroOffset[2];

  //时间间隔 ms
  interval = (millis() - preInterval) * 0.001;

  //根据陀螺仪的角速度和时间间隔更新角度值。
  angleGyro[0] += gyro[0] * interval;
  angleGyro[1] += gyro[1] * interval;
  angleGyro[2] += gyro[2] * interval;

  //融合加速度和陀螺仪角度
  //使用加权平均（融合算法）将陀螺仪和加速度计的角度值融合在一起，以获得更精确和稳定的角度值。其中gyroCoef和accCoef是两个权重系数，表示对陀螺仪和加速度计数据的信任程度。
  angleRPY[0] = (gyroCoef * (angleRPY[0] + gyro[0] * interval)) + (accCoef * angleAcc[0]);//Roll
  angleRPY[1] = (gyroCoef * (angleRPY[1] + gyro[1] * interval)) + (accCoef * angleAcc[1]);//Pitch
  angleRPY[2] = angleGyro[2];//yaw

  //更新上次更新时间
  preInterval = millis();
}


void SF_IMU::writeToIMU(uint8_t addr, uint8_t data) {
  _i2c->beginTransmission(MPU6050_ADDR);
  _i2c->write(addr);
  _i2c->write(data);
  _i2c->endTransmission();
}

uint8_t SF_IMU::readFromIMU(uint8_t addr) {
  _i2c->beginTransmission(MPU6050_ADDR);
  _i2c->write(addr);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  return _i2c->read();
}