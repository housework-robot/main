# Anatomy of the Motion Control of Mushibot

## 1. Objectives

In [the previous blog](./S06E04_anatomy_wheel_legged_mushibot.md), we studied how to wire the motors, servos, sensors and IMU modules together, 
to construct a wheel-legged robot, [Mushibot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot), 
which is almost a mini version of ETH Zurich's Ascento, 
with some differences in the shape and structure of the leg linkages etc.

In this blog, we dive into the details how to control the motors and servos for desired motion, using the robot status information collected from 
the motor encoders, the IMU (gyro and accelerometer) modules and ADC (analog to digital converter). 
Later on, we will discuss how to use the environmental information retrieved from camera etc. 

&nbsp;
## 2. Analog to Digital Converter with ESP32 

The APIs of ADC with ESP32 changes dramatically from the previous versions to the latest v5.3.x. 

The general concept of ADC is quite straightward, 
according to [Wikipedia](https://en.wikipedia.org/wiki/Analog-to-digital_converter),

> An ADC provides an isolated measurement such as an electronic device that converts an analog input voltage or current
> to a digital number representing the magnitude of the voltage or current.

There are a few technical terms that we must understand in order to use the APIs of ESP32 ADC correctly.

&nbsp;
### 2.1 Terminology

[The purpose of ADC](https://www.arrow.com/en/research-and-events/articles/engineering-resource-basics-of-analog-to-digital-converters) 
is to convert a analog continuous signal into a digital signal, referring to the following picture. 

   <p align="center">
     <img alt="the purpose of ADC" src="./S06E05_src/images/purpose_of_adc.png" width="85%">
   </p>

#### 1. [Sampling rate/Frequency](https://www.arrow.com/en/research-and-events/articles/engineering-resource-basics-of-analog-to-digital-converters)

The sampling rate or frequency of an ADC is explained in the following picture. 
The higher rate, the better accuracy. 

   <p align="center">
     <img alt="the sampling rate / frequency of ADC" src="./S06E05_src/images/sampling_rate_adc.png" width="60%">
   </p>

#### 2. [Resolution/Bitwidth](https://www.arrow.com/en/research-and-events/articles/engineering-resource-basics-of-analog-to-digital-converters)

The resolution of an ADC depends on the number of logic gates inside the ADC chip. 
The more logic gates, the higher resolution, the better accuracy. 

The resolution of an ADC is usually represented by *bitwidth*. 

   <p align="center">
     <img alt="the resolution / bitwidth of ADC" src="./S06E05_src/images/resolution_adc.png" width="60%">
   </p>

#### 3. [Vref vs Step size](https://eevibes.com/computing/discuss-the-characteristics-of-adc-in-detail/)

`Vref` stands for reference voltage. It is one of the input voltages. 

`Step size` is determined by reference voltage and resolution. In an 8-bit ADC, step size is Vref/256 because 2 to the power of 8 give us 256 steps. The larger reference voltage we give as input, we get larger value of step size.

#### 4. [Attenuation](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html#adc-attenuation)

`Vref` is the reference voltage used internally by ESP32 ADCs for measuring the input voltage. 

The ESP32 ADCs can measure analog voltages from `0` V to `Vref`. Among different chips, the `Vref` varies, the median is 1.1 V. 

In order to convert voltages larger than `Vref`, input voltages can be attenuated before being input to the ADCs. 

There are 4 available attenuation options, the higher the attenuation is, the higher the measurable input voltage could be.

- `ADC_ATTEN_DB_0`: 100 mV ~ 950 mV
- `ADC_ATTEN_DB_2_5`: 100 mV ~ 1250 mV
- `ADC_ATTEN_DB_6`: 150 mV ~ 1750 mV
- `ADC_ATTEN_DB_12`: 150 mV ~ 2450 mV

#### 5. [Conversion](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html#adc-conversion)

An ADC conversion is to convert the input analog voltage to a digital value. The ADC conversion results provided by the ADC driver APIs are raw data. 

~~~
adc1_get_raw()
adc2_get_raw()
~~~

Resolution of ESP32 ADC raw results under Single Read mode is 12-bit.

To calculate the voltage based on the ADC raw results, this formula can be used,

~~~
Vout = Dout * Vmax / Dmax 
~~~

- Dout: ADC raw digital reading result, i.e. the result of `adc1_get_raw()`.
- Vmax: Maximum measurable input analog voltage, i.e. `Vref`. 
- Dmax: Since the resolution of ESP32 ADC is 12-bit, 2^12 = 4096, the maximum of the output ADC raw digital reading result, Dmax = 4095.

#### 6. Parallel vs Serial ADC

#### 7. One-shot vs Continuous ADC

#### 8. Channel 

#### 9. ADC1 vs ADC2

#### 10. Calibration scheme

#### 11. Calibration values

#### 12. eFuse for calibration values

#### 13. characteristics



&nbsp;
### 2.2 Workflow









