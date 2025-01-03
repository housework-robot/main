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

#### 1. Sampling rate/Frequency

The sampling rate or frequency of an ADC is explained in the following picture. 
The higher rate, the better accuracy. 

   <p align="center">
     <img alt="the sampling rate / frequency of ADC" src="./S06E05_src/images/sampling_rate_adc.png" width="60%">
   </p>

#### 2. Resolution/Bitwidth

The resolution of an ADC depends on the number of logic gates inside the ADC chip. 
The more logic gates, the higher resolution, the better accuracy. 

The resolution of an ADC is usually represented by *bitwidth*. 

   <p align="center">
     <img alt="the resolution / bitwidth of ADC" src="./S06E05_src/images/resolution_adc.png" width="60%">
   </p>

#### 3. Vref, Reference voltage

#### 4. Attenuation 

#### 5. Conversion

#### 6. Parallel vs Serial ADC

#### 7. One-shot vs Continuous ADC

#### 8. Channel

#### 9. Calibration scheme

#### 10. Calibration values

#### 11. eFuse for calibration values

#### 12. characteristics



&nbsp;
### 2.2 Workflow









