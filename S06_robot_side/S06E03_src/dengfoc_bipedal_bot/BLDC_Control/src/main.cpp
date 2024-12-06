#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SF_Motor.h"
#include "SF_Communication.h"
#include "Pins_Specify.h"

//串口发送的

SPIClass vspi(VSPI);//如果用到MT6701编码器，采用的SPI通信，需要实例化SPI总线
TwoWire iic0 = TwoWire(0);//如果是AS5600编码器，采用IIC通信，双电机控制需要实例化两条IIC总线
TwoWire iic1 = TwoWire(1);

SF_Motor M0 = SF_Motor(0);//实例化电机类
SF_Motor M1 = SF_Motor(1);

SF_Communication com = SF_Communication();//实例化通信接口

float Vbus = 12.0;//设置供电电压值
float alignVoltage = 3;//电机-编码器校准时的电压值

void setup()
{
    Serial.begin(115200);
    com.linkMotor(M0, M1); // 链接电机和通信接口 可以选一个或两个
    com.init(ONBOARD); //选择通信的输出 USB:通过USB输出；ONBOARD:与扩展主控通信。
    // com.init(USB);

    //为AS5600编码器的初始化
    // iic0.begin(AS5600_SDA0, AS5600_SCL0, 400000UL);
    // iic1.begin(AS5600_SDA1, AS5600_SCL1, 400000UL);
    // M0.initEncoder(AS5600,iic0);
    // M1.initEncoder(AS5600,iic1);
    //为MT6701编码器的初始化
    vspi.begin(MT6701_CLK, MT6701_DO, 0, -1);
    M0.initEncoder(MT6701, vspi);
    M1.initEncoder(MT6701, vspi);

    //电机初始化，传入电压值
    M0.init(Vbus);
    M1.init(Vbus);
    //电机-编码器的校准，传入校准时的电压值
    M0.AlignSensor(alignVoltage);
    M1.AlignSensor(alignVoltage);

    //通信端口开启持续的电机状态输出
    com.start();

    //设置相关的PID值 传入参数为P、I、D、Limit值
    M0.setAnglePID(0.5, 0, 0, 6);
    M0.setVelPID(0.05, 0.005, 0, 6);
    M0.setCurrentPID(1.2,0,0,0);

    M1.setAnglePID(0.5, 0, 0, 6);
    M1.setVelPID(0.05, 0.005, 0, 6);
    M1.setCurrentPID(1.2,0,0,0);
    
}

uint32_t now_time = 0;
uint32_t last_time = 0;

void loop()
{
  //电机的循环控制执行
  M0.run();
  M1.run();
  

  //设置电机的控制方式示例,当然也可以通过串口控制
  M0.setTorque(1);
  M1.setTorque(1);
  M0.setForceAngle(0);
  M1.setForceAngle(0);
  
  M0.setVelocity(10);
  M1.setVelocity(10);
}
