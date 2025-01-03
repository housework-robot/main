# Anatomy of the ADC reading of Mushibot

## 1. Objectives

In [the previous blog](./S06E04_anatomy_wheel_legged_mushibot.md), we studied how to wire the motors, servos, sensors and IMU modules together, 
to construct a wheel-legged robot, [Mushibot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot), 
which is almost a mini version of ETH Zurich's Ascento, 
with some differences in the shape and structure of the leg linkages etc.

In this blog, we dive into the details how to read calibrated voltage from ADC. 

Later on we will disucss how to control the motors and servos for desired motion, using the robot status information collected from 
the motor encoders, the IMU (gyro and accelerometer) modules and ADC (analog to digital converter). 

After that, we will discuss how to use the environmental information retrieved from camera etc. 

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

Resolution of [ESP32 ADC](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html#adc-conversion) raw results is 12-bit.

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

#### 6. [Channel, ADC1 & ADC2](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html#adc-channels)

The ESP32 integrates 2 SAR (Successive Approximation Register) ADCs, supporting a total of 18 measurement channels (analog enabled pins).

- `ADC1`, 8 channels: GPIO32 - GPIO39
  
- `ADC2`, 10 channels: GPIO0, GPIO2, GPIO4, GPIO12 - GPIO15, GOIO25 - GPIO27

The `ADC2` is mainly used by wifi, hence in most cases, we only use `ADC1`. 


#### 7. [One-shot vs Continuous ADC](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/peripherals/adc_oneshot.html#introduction)

ADC can be used in two scenarios, 

- Generate one-shot ADC conversion result

- Generate continuous ADC conversion results

Both of the ADC units support one-shot mode, which is suitable for low-frequency sampling operations.

#### 8. [Parallel vs Serial ADC](https://eevibes.com/computing/discuss-the-characteristics-of-adc-in-detail)

There are two types of ADC i.e. parallel and serial ADC. 

Parallel ADC gives output in chunks. They have 8 pins for output, D0-D7 gives output between ADC and the CPU in an 8 bit ADC. 

It is a faster way of getting digital values from analog signal. But it takes too much place on a circuit board. 

When circuit space is crucial, we use serial ADC. 

Serial ADC is a bit slower in giving output than parallel ADC, because it gives one bit at a time as output, 
hence we have only one pin for output. 

Due to this, serial ADCs are widely used in circuits. 


#### 9. [Calibration scheme](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html#adc-calibration)

Even though the design value of the ADC reference voltage is 1100 mV, 
the true reference voltage can range from 1000 mV to 1200 mV amongst different ESP32 chips.

Calibration is to align the input voltage with the ADC reading, regardless the various true reference voltages of different ESP32 chips.

To align the input voltage with the ADC reading, we can use line-fitting or curve-fitting [calibration scheme](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/peripherals/adc_calibration.html#adc-calibration-line-fitting-scheme). 

The line or the curve of the input voltage with respect to the ADC reading, is called `characteristic curve`.

The following picture displays two ESP32 chips with different real reference voltages. 
With different reference voltages, the alignments of the input voltage with the ADC reading are different. 

The picture displays the two line-fittings with respect to the two different reference voltages. 

   <p align="center">
     <img alt="the alignments of the input voltage with the ADC reading, with linear fitting scheme" src="./S06E05_src/images/calibration_adc.jpg" width="60%">
   </p>


#### 10. [Calibration values](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html#calibration-values)

Calibration values are used to generate the `characteristic curves` that account for the variation of ADC reference voltage of a particular ESP32 chip. 

For line-fitting scheme, its calibration values are used to generate the parameters `a` and `b` in the linear formular, `y = a * x + b`. 

There are currently 3 sources of calibration values on ESP32,

- Two Point values:

   They represent each of the ADCs’ readings at 150 mV and 850 mV. To obtain more accurate calibration results, these values should be measured by user and burned into eFuse `BLOCK3`.

- eFuse Vref:

  This value represents the true ADC reference voltage. It is measured and burned into eFuse `BLOCK0` during factory calibration.

- Default Vref:

  This value is an estimate of the ADC reference voltage, provided by the user as a parameter during characterization. If `Two Point` or `Vref` values are unavailable, `Default Vref` will be used.

#### 11. [eFuse for calibration values](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/peripherals/adc_calibration.html#adc-calibration-line-fitting-scheme)

In ESP32 chips, `eFuse` is a memory component to store important data safely, and perhaps permanently. 

One can use `adc_cali_scheme_line_fitting_check_efuse()` to check the eFuse bits. 

Normally, the line-fitting scheme eFuse value is `ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP` or `ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF`. 
This means the Line Fitting scheme uses calibration parameters burned in the eFuse to do the calibration.

#### 12. [Characteristics](https://eevibes.com/computing/discuss-the-characteristics-of-adc-in-detail)

As mentioned above, `characteristic curve` refers to the line or the curve of calibration scheme, for the alignment of the input voltage with the ADC reading. 

However, `characteristics of an ADC` is a general term containing the following contents, 

- Resolution, 
- Conversion time, 
- Vref, 
- Digital data output, `Dout = Vin / Step_size`,
- Parallel versus serial ADC, 
- Analog input channels, 
- Start-conversion and end-of-conversion signals. 



&nbsp;
### 2.2 Workflow

[`oneshot_read_main.c`](https://github.com/espressif/esp-idf/blob/v5.3.2/examples/peripherals/adc/oneshot_read/main/oneshot_read_main.c)
is a sample code for the usage of ESP32's ADC APIs for version 5.3.x. 

Essentially, the workflow to get calibrated voltage from ESP32's ADC consists of the following steps, 

1. Initialize an instance of the ADC1's handler, with configuration setting.
   
2. Initialize an instance of a ADC1's channel, with configuration setting.

   In case you need to access multiple ADC channels, you can initialize multiple channel instances, one instance for one channel.

3. Initialize an instance of a ADC1's calibration for one channel, with configuration setting.

   In case you have multiple channels, you can initialize multiple calibration instances, one calibration instance for one channel. 

4. Read the raw data from ADC1, using the ADC1's handler and the channel instance.

5. Do the calibration of the raw data, using the ADC1's calibration instance.

6. Delete the ADC1's handler instance and its calibration instances.

In case you need to access the channels and calibrations of ADC2, repeat the above 6 steps for ADC2.

&nbsp;
#### 1. Mushibot class 

In our scenario, we only use ADC1, and reserve ADC2 for wifi. 

We reconstructed Mushibot's original source code, to make it more readable and easier to maintain. 

We define a `Mushibot` class, that contains multiple components including ADC. 

~~~
#include <esp_log.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>

class Mushibot
{
public:
    Mushibot();
    ~Mushibot();
    void setup_mushibot();
    void loop_mushibot();

    int get_voltage(); 
    void bat_check();
    ...

private:  
    // ADC
    // We don't explicitly select the input pin 35 for the ADC,
    // but rather using the channel, GPIO35 == Channel 7.
    // int BAT_PIN = 35;

    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle;
    bool is_calibrated = false;

    void setup_adc();
};
~~~

Notice that, we don't directly read from ADC1's pin, that is GPIO35. 
Instead, we read the ADC1 raw data from channel 7, which is identical to GPIO35, 
and aligned with the official usages of ESP32 ADC's APIs. 

Referring to [the pinout of ESP32 chip](https://github.com/MuShibo/Micro-Wheeled_leg-Robot/blob/master/2.Hardware/1.ControllerPCB/Schematic.pdf) 
in the Mushibot controller board, ADC1 gets its Vin from channel 7, which is identical to GPIO35.

   <p align="center">
     <img alt="Channel 7 is identical to GPIO35 in Mushibot's controller board" src="./S06E05_src/images/ESP32_chip_pinout.png" width="80%">
   </p>


&nbsp;
#### 2. ADC setup

~~~
void Mushibot::setup_adc() {
    esp_err_t ret = ESP_FAIL;

    // ADC1 handle
    adc_oneshot_unit_init_cfg_t adc1_handle_init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(
        &adc1_handle_init_config, 
        &adc1_handle
    ));

    // ADC1 channel
    adc_oneshot_chan_cfg_t adc1_handle_channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc1_handle, 
        ADC_CHANNEL_7, 
        &adc1_handle_channel_config
    ));

    // ADC1 calibration 
    if (!is_calibrated) {
        Serial.printf("\n[INFO] Calibration scheme is Line-fitting. \n");
        adc_cali_line_fitting_config_t adc1_cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };

        ret = adc_cali_create_scheme_line_fitting(
            &adc1_cali_config, 
            &adc1_cali_handle
        );
        if (ret == ESP_OK) {
            is_calibrated = true;
            Serial.printf("[INFO] Calibration succeed.\n");
        }
        else if (ret == ESP_ERR_NOT_SUPPORTED || !is_calibrated) {
            Serial.printf("[WARN] eFuse not burned, skip software calibration.\n");
        }
        else {
            Serial.printf("[WARN] Invalid arguements or no memory.\n");
        }
    }

    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_calibration.html
    adc_cali_line_fitting_efuse_val_t efuse_tp = ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP; 
    adc_cali_line_fitting_efuse_val_t efuse_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF; 

    // Check TP is burned into eFuse, not quite useful, feel safe to delete.
    if (adc_cali_scheme_line_fitting_check_efuse(&efuse_tp) == ESP_OK) {
        Serial.printf("\n[INFO] eFuse Two Point: Supported. \n");
    } else {
        Serial.printf("\n[INFO] eFuse Two Point: NOT supported. \n");
    }

    // Check Vref is burned into eFuse
    if (adc_cali_scheme_line_fitting_check_efuse(&efuse_vref) == ESP_OK) {
        Serial.printf("[INFO] eFuse Vref: Supported. \n");
    } else {
        Serial.printf("[INFO] eFuse Vref: NOT supported. \n");
    }
}
~~~

- In the above source code, when we set the configuration of the ADC1's handle, its channels and the calibration,
  we use the default parameters, including
  `ADC_UNIT_1`, `ADC_ULP_MODE_DISABLE`, `ADC_ATTEN_DB_12`, `ADC_BITWIDTH_DEFAULT`, `ADC_CHANNEL_7` etc.

- We use line-fitting scheme for calibration, rather than curve-fitting scheme for simplicity.

- In fact, it is not necessary to check the eFuse for `ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP` for two-points
  and `ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF` for vRef,

  But it doesn't hurt either. We do the eFuse check in the setup procedure for debugging purpose.


&nbsp;
#### 3. ADC reading

~~~
int Mushibot::get_voltage() {
    int adc_raw;
    int adc_voltage;
  
    // Read raw data from ADC
    ESP_ERROR_CHECK(adc_oneshot_read(
        adc1_handle,
        ADC_CHANNEL_7, 
        &adc_raw
    ));
    Serial.printf("\n[INFO] ADC%d channel[%d] raw data: %d. \n",
        ADC_UNIT_1 + 1, ADC_CHANNEL_7, adc_raw);

    // Read calibrated data from ADC
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(
        adc1_cali_handle, 
        adc_raw,
        &adc_voltage
    ));
    Serial.printf("[INFO] ADC%d channel[%d] calibrated voltage: %d mV. \n",
        ADC_UNIT_1 + 1, ADC_CHANNEL_7, adc_voltage);
    
    return adc_voltage;
}


void Mushibot::bat_check() {
    uint32_t sum = 0;
    int calibrated_voltage; 
    float battery;

    if (bat_check_num > 1000) {
        // sum = analogRead(BAT_PIN);
        // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, sum, &calibrated_voltage));
        calibrated_voltage = get_voltage();
        battery = (float) calibrated_voltage;
        battery = (battery * 3.97) / 1000.0;
        Serial.printf("[INFO] Battery is %.2f. \n", battery);
        
        //电量显示
        if (battery > 7.8)
            digitalWrite(LED_BAT, HIGH);
        else
            digitalWrite(LED_BAT, LOW);

        bat_check_num = 0;
    } else
        bat_check_num++;  
}
~~~

Notice that the battery check `bat_check()` calls `get_voltage()` to get the calibrated voltage from pin GPIO35, 
which is equivalent to channel 7. 

The battery level is equal to `(calibrated_voltage * 3.97) / 1000.0`. This is copied from Mushibot's original source code. 


&nbsp;
## 3. Demo

As mentioned above, we reconstructed Mushibot's original source code, to make it more readable and easier to maintain. 

[Our source code](./S06E05_src/src/README.md) is stored in this repo. 

Click the following image to view the video of Mushibot's jumping. 

   [![The outlook of the Stack-force wheel-legged bot](https://img.youtube.com/vi/knGd_0lGek0/hqdefault.jpg)](https://www.youtube.com/shorts/knGd_0lGek0)





