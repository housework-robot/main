#include <Arduino.h>
#include "SF_Servo.h"
#include "Wire.h"

SF_Servo servos = SF_Servo(Wire);

int16_t servo1Offset,servo2Offset,servo3Offset,servo4Offset;

void setServosAngle(int16_t val1,int16_t val2,int16_t val3,int16_t val4){
  servos.setAngle(3, 180+val1);
  servos.setAngle(4, 180+val2);
  servos.setAngle(5, 180+val3);
  servos.setAngle(6, 180+val4);
}


void read(){
  if (Serial.available() > 0) {
    // 读取串口输入的数据，按逗号分隔
    servo1Offset = Serial.readStringUntil(',').toInt();
    servo2Offset = Serial.readStringUntil(',').toInt();
    servo3Offset = Serial.readStringUntil(',').toInt(); 
    servo4Offset = Serial.readStringUntil('\n').toInt(); // 最后一个数以换行结尾
  }
}

void setup(){
  Serial.begin(115200);
  Wire.begin(1, 2, 400000UL);
  servos.init();
  servos.setAngleRange(0,300);
  servos.setPluseRange(500,2500);

}


void loop() {
  // put your main code here, to run repeatedly:
  read();
  Serial.printf("%d, %d, %d, %d\n",servo1Offset,servo2Offset,servo3Offset,servo4Offset);
  setServosAngle(servo1Offset,servo2Offset,servo3Offset,servo4Offset);//四个舵机的舵值
  delay(1000);
}



