#ifndef _SF_BLDC_h
#define _SF_BLDC_h

#include <Arduino.h>
#include "SF_BLDC_shared_struct.h"

class SF_BLDC{
    public:
        SF_BLDC(HardwareSerial &serial);

        // char Command_1,Command_7;    //包头和包尾
        // float Command_2,Command_3,Command_4,Command_5,Command_6;    //中间指令

        uint8_t M0_mode, M1_mode;
        float M0_target, M1_target;


        void init();
        SF_BLDC_DATA getBLDCData();
        void setTargets(float M0_tar,float M1_tar);
        void setModes(uint8_t M0_mod, uint8_t M1_mod);
        void setM0VelPID(float P,float I,float D,float limit);
        void setM1VelPID(float P,float I,float D,float limit);
        void setM0AnglePID(float P,float I,float D,float limit);
        void setM1AnglePID(float P,float I,float D,float limit);
        void setM0CurrentPID(float P,float I,float D,float limit);
        void setM1CurrentPID(float P,float I,float D,float limit);

        void alignSensors(float M0_AlignVoltage,float M1_AlignVoltage);



    private:
        HardwareSerial *_serial;
        String receivedChars;
        float uart_temp_M0_angle,uart_temp_M1_angle,uart_temp_M0_vel,uart_temp_M1_vel,uart_temp_M0_Uq,uart_temp_M1_Uq,uart_temp_M0_Ud,uart_temp_M1_Ud,uart_temp_M0_Iq,uart_temp_M1_Iq,uart_temp_M0_Id,uart_temp_M1_Id;
        float recM0Angle,recM1Angle,recM0Vel,recM1Vel,recM0Uq,recM1Uq,recM0Ud,recM1Ud,recM0Iq,recM1Iq,recM0Id,recM1Id;
        SF_BLDC_DATA values;
        float M0_VelP,M0_VelI,M0_VelD;
        float M1_VelP,M1_VelI,M1_VelD;
        float M0_AngleP,M0_AngleI,M0_AngleD;
        float M1_AngleP,M1_AngleI,M1_AngleD;
        float M0_CurrentP,M0_CurrentI,M0_CurrentD;
        float M1_CurrentP,M1_CurrentI,M1_CurrentD;

        static void appCpuLoopWrapper(void *pvParameters); // 静态包装函数
        void appCpuLoop(); // 实际的成员函数
        String recFromSerial();
        void sendToSerial(String S_Com_1,float S_Com_2,float S_Com_3,float S_Com_4,float S_Com_5,float S_Com_6,String S_Com_7);

};


#endif