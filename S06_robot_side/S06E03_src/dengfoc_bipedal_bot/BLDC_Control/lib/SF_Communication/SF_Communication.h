#ifndef SF_COMMUNICATION_H
#define SF_COMMUNICATION_H
#include <Arduino.h>

/*-----通信方式说明-------------
T[M0目标值],[M1目标值],[M0控制模式],[M1控制模式],0B  举例：T10,10,1,1,0B M0M1电机在速度模式下以10rad/s旋转
C[设置的对象电机],[电流环P值],[电流环I值],[电流环D值],[电流环限幅值]T
V[设置的对象电机],[速度环P值],[速度环I值],[速度环D值],[速度限幅值]Y
A[设置的对象电机],[位置环P值],[位置环I值],[位置环D值],[位置环限幅值]E
U[M0校准电压值],[M1校准电压值],0,0,0V
-----------------------------*/
class SF_Motor;

#define NOCONTACT 0
#define USB 1
#define ONBOARD 2

class SF_Communication
{
public:
    SF_Communication();
    void setUSBBaud(uint32_t baud);//设置USB通信时的波特率
    void init(uint8_t contactSerial);//初始化 传入选择的通信对象
    void stop();
    void start();

    void linkMotor(SF_Motor &motor0, SF_Motor &motor1);
    void linkMotor(SF_Motor &motor0);

private:
    HardwareSerial *_serial;
    uint8_t _contactSerial;
    SF_Motor *M0, *M1;
    uint32_t _USBbaud;
    bool _startFlag;
    char _recCommand1,  _recCommand7;
    float _recCommand2, _recCommand3, _recCommand4, _recCommand5, _recCommand6;
    int _commaPosition;
    uint16_t sendLoopCount;

    static void taskFunction(void *parameter)
    {
        SF_Communication *instance = static_cast<SF_Communication *>(parameter);

        while (true)
        {
            instance->appCpuLoop();
            vTaskDelay(1); // 延时1ms
        }
    }

    void appCpuLoop();
    void sendMotorStatus0();
    void sendMotorStatus1();
    void sendMotorStatus2();
    void recCommand();
    void calRecCommand();

    TaskHandle_t taskHandle; // 任务句柄
};

#endif
