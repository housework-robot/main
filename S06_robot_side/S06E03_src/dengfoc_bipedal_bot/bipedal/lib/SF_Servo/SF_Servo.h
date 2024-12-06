#ifndef _SF_SERVO_h
#define _SF_SERVO_h

#include <Arduino.h>
#include <Wire.h>



#define SERVO_ENABLE_PIN 42
#define PCA9685_ADDR 0x40

#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */

#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02                    /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      //默认的PCA9685 I2C从地址
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3    //最小预调值
#define PCA9685_PRESCALE_MAX 255  //最大预调值

class SF_Servo
{
  public:
    SF_Servo(TwoWire &i2c);
    void init();
    void enable();
    void disable();
    void setAngle(uint8_t num, uint16_t angle);
    void setAngleRange(uint16_t min, uint16_t max);
    void setPluseRange(uint16_t min, uint16_t max);
    void setPWMFreq(float freq);
    void setPin(uint8_t num, uint16_t val, bool invert = false);
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    void reset();
    void sleep();
    void wakeup();
      

  private:
    uint16_t angleMin,angleMax;
    uint16_t pluseMin, pluseMax;
    uint16_t angleRange, pluseRange;
    uint8_t freq;
    float cal_min;
    float cal_max;

    TwoWire *_i2c;

    void writeToPCA(uint8_t addr, uint8_t data);
    uint8_t readFromPCA(uint8_t addr);


    
};

#endif