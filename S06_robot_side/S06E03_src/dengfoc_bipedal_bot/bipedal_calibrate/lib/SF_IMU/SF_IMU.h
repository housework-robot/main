#ifndef _SF_IMU_h
#define _SF_IMU_h

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42

class SF_IMU{
    public:
        SF_IMU(TwoWire &i2c);

        
        float temp, acc[3], gyro[3];
        float angleGyro[3],angleAcc[2];
        float angle[3];
        


        void init();
        void calGyroOffsets();
        void update();
        void setGyroOffsets(float x, float y, float z);
        void setCoef(float accCoef, float gyroCoef);
        // float* getAcc();
        // float* getGyro();
        // float getTemp();
        // float* getAngleAcc();
        // float* getAngleGyto();
        // float* getAngle();


    
    private:
        TwoWire *_i2c;
        float _accCoef, _gyroCoef;
        float _gyroOffset[3];
        // float gyroXoffset, gyroYoffset, gyroZoffset;
        // float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
        // float angleGyroX, angleGyroY, angleGyroZ,angleAccX, angleAccY, angleAccZ;
        // float angleX, angleY, angleZ;
        float _interval, _preInterval;

        void writeToIMU(uint8_t addr, uint8_t data);
        uint8_t readFromIMU(uint8_t addr);

};



#endif