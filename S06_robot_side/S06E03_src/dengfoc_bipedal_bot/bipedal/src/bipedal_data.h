#ifndef _BIPEDAL_DATA_h
#define _BIPEDAL_DATA_h

#define SBUSPIN 41 // not completely conform

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数
//0
#define RCCHANNEL_MIN 600
#define RCCHANNEL_MAX 1400  

#define RCCHANNEL3_MIN 200
#define RCCHANNEL3_MID 1000
#define RCCHANNEL3_MAX 1800

#define LFSERVO_CH 3 // todo
#define LRSERVO_CH 4
#define RFSERVO_CH 5
#define RRSERVO_CH 6

#define ROBOTMODE_DIABLE 0
#define ROBOTMODE_MOTION 1
#define ROBOTMODE_CALIBRATION 2

#define ROBOT_HIGHEST 130
#define ROBOT_LOWEST_FOR_MOT 70
#define ROBOT_LOWESR_FOR_CAL 70

#define L1  60
#define L2  100
#define L3  100
#define L4  60
#define L5  40
#define L6  0
// #define PI  3.1415926
//-9,-6,-3,-5
#define LFSERVO_OFFSET 0
#define LRSERVO_OFFSET 5
#define RFSERVO_OFFSET 4
#define RRSERVO_OFFSET 3


int RCValue[6];


typedef struct 
{
    float pitch;
    float roll;
    float yaw;
    float GyroX;
    float GyroY;
    float GyroZ;

    float height;
    float speedAvg; //0805 ming 新增平均速度
}robotposeparam;

typedef struct 
{
    uint8_t mode;

    float forward;//会用在车轮控制与逆运动学逆解
    float turn;//仅用于运动学逆解
    float updown;//仅用于运动学逆解
    float roll;//仅用于运动学逆解

    float throttleLimit; //油门限制
    float turnLimit;// 转向限制
    float forwardLimit;//前后倾限制
    float rollLimit;//侧倾限制
    uint8_t heightest;//高度最高限制
    uint8_t lowest;//高度最低限制
}robotmotionparam;

typedef struct {
    uint8_t mode;
    bool printFlag;

    bool motorEnable;
    bool servoEnable;
}robotmode;

typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
}PIDIncrement;

typedef struct 
{
    uint16_t servoLeftFront;
    uint16_t servoLeftRear;
    uint16_t servoRightFront;
    uint16_t servoRightRear;
    float motorLeft;
    float motorRight;
}motorstarget;

typedef struct{
    float M0Speed;
    float M1Speed;
    int M0Dir, M1Dir;
    int M0SpdDir, M1SpdDir;
}motorstatus;

typedef struct{
    float height;
    float legRollLeft;
    float legRollRight;
    float legLeft;
    float legRight;
    float roll;
    float forward;
    float pitch;
    float velocity;
    float differVel;
    float centerAngleOffset;
}controlparam;


typedef struct{
    float x;
    float xLeft,xRight;
    float yLeft, yRight;
}coordinate;

typedef struct{
    float alpha1, beta1;
    float alpha2, beta2;
    float XLeft,YLeft;
    float XRight, YRight;
}IKparam;

#endif // 