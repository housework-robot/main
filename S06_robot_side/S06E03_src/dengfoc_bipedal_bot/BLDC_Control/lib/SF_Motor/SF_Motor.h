#ifndef _SF_MOTOR_h
#define _SF_MOTOR_h

#include <Arduino.h>

class SF_Enc;
class SF_FOC;
class PIDController;
class LowPassFilter;
class SPIClass;
class TwoWire;

struct MotorParam
{
    float polePairs;
    float phaseR;
    float zeroElecAngle;
    int dir;
};

#define VELOCITY_MODE 1
#define FORCE_ANGLE_MODE 2
#define VEL_ANGLE_MOED 3
#define TORQUE_MODE 4

#define NOT_SET -12345.0
#define _isset(a) ((a) != (NOT_SET))

class SF_Motor
{
    public:
        SF_Motor(uint8_t motorNum);//
        ~SF_Motor();
        void init(float Vbus);//初始化
        void initEncoder(uint8_t encoderType,SPIClass &spi);//重载 主要给MT6701 用到SPI通信的
        void initEncoder(uint8_t encoderType,TwoWire &iic);//重载 主要给AS5600 用到IIC通信的
        void initCurrentSense();
        void setCurrentSense(bool torqueMode);
        void AlignSensor(float voltageSensorAlign);
        void setMotorParam(float polePairs,float dir,float phaseR, double zeroElecAngle);//手动设置电机参数

        void run();//run FOC

        //传感器数据获取
        float getAnlge();//获取电机角度 单位rad
        float getVelocity();//获取电机速度 单位rad
        float getCurrent();//获取电流值 单位mA
        float getElecAngle();//获取电机电角度

        void setTorque(float target);//设置目标扭矩A （会自动将电机设置至力矩控制模式）
        void setVelocity(float target);//设置目标速度rad/s （会自动将电机设置至速度控制模式）
        void setForceAngle(float target);//设置目标角度rad（力位模式） （会自动将电机设置至位置控制模式）
        void setVelocityAngle(float target);//设置目标角度rad（力速位模式） （会自动将电机设置至力矩控制模式）

        void setVelPID(float P,float I,float D,float limit);//设置速度环PID，用于速度控制，力速位的位置控制
        void setAnglePID(float P,float I,float D,float limit);//设置角度环PID，用于力位、力速位的位置控制
        void setCurrentPID(float P,float I,float D,float limit);//设置电流环PID

        float UqOutput, UdOutput;
        
        float target;
        uint8_t controlMode;

    private:  
        SF_FOC *FOC;
        SF_Enc *Enc;
        // SF_Communication *com;

        uint8_t edition;
        float _voltageSupply;
        uint8_t _torqueMode;
        uint8_t _encoderType;
        uint8_t _motorNum;
        MotorParam _motorParam;
        


        PIDController *angleLoop, *velLoop, *currentLoop;
        LowPassFilter *velFlt, *currentFlt;

        float velPID(float error);
        float anglePID(float error);
        float currentPID(float error);

};




#endif