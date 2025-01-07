#include "mushibot.h"

Mushibot::Mushibot() {
 
};

Mushibot::~Mushibot()
{
    /*
    delete &motor1;
    delete &motor2;
    delete &driver1;
    delete &driver2;
    
    delete &I2Cone;
    delete &I2Ctwo;
    delete &sensor1;
    delete &sensor2;

    delete &sms_sts;
    delete &mpu6050;
    */
}


void Mushibot::setup_servo() {
    //舵机初始化
    sms_sts.pSerial = &Serial2;

    byte ID[2];
    ID[0] = 1;
    ID[1] = 2;

    byte ACC[2];
    ACC[0] = 30;
    ACC[1] = 30;

    //舵机有效行程450
    u16 Speed[2];
    Speed[0] = 300;
    Speed[1] = 300;

    //左侧舵机[2048+12+50,2048+12+450]
    //左侧舵机[2048-12-50,2048-12-450]
    s16 Position[2];
    Position[0] = 2148;
    Position[1] = 1948;

    s16 original_Position[2];
    original_Position[0] = 0;
    original_Position[1] = 0;

    pinMode(LED_BAT, OUTPUT); 
    digitalWrite(LED_BAT, HIGH);

    sms_sts.SyncWritePosEx(ID, 2, original_Position, Speed, ACC);
    delay(1000);

    //舵机(ID1/ID2)以最高速度V=2400步/秒，加速度A=50(50*100步/秒^2)，运行至各自的Position位置
    // Serial.printf("\n[DEBUG] Before sms_sts.SyncWritePosEx(), Speed=(%d, %d), Position=(%d, %d), ACC=(%d, %d) \n", 
    //     Speed[0], Speed[1], Position[0], Position[1], ACC[0], ACC[1]);
    sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
    delay(1000);

    digitalWrite(LED_BAT, LOW);
    Serial.printf("\n[INFO] setup_servo() is done. \n");
}


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
        Serial.printf("\n[INFO] eFuse TwoPoint: Supported. \n");
    } else {
        Serial.printf("\n[INFO] eFuse TwoPoint: NOT supported. \n");
    }

    // Check Vref is burned into eFuse
    if (adc_cali_scheme_line_fitting_check_efuse(&efuse_vref) == ESP_OK) {
        Serial.printf("[INFO] eFuse Vref: Supported. \n");
    } else {
        Serial.printf("[INFO] eFuse Vref: NOT supported. \n");
    }

    Serial.printf("\n[INFO] setup_adc() is done. \n");
}


int Mushibot::get_voltage() {
    //电压检测
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


//电压检测
void Mushibot::bat_check() {
    uint32_t sum = 0;
    int calibrated_voltage; 
    float battery;

    if (bat_check_num > 1000) {
        //电压读取
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


void Mushibot::setup_mushibot() {
    setup_servo();
    setup_adc();

    // 编码器设置
    I2Cone.begin(19, 18, 400000UL);
    I2Ctwo.begin(23, 5, 400000UL);
    sensor1.init(&I2Cone);
    sensor2.init(&I2Ctwo);

    //mpu6050设置
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    //连接motor对象与编码器对象
    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);

    //速度环PID参数
    motor1.PID_velocity.P = 0.05;
    motor1.PID_velocity.I = 1;
    motor1.PID_velocity.D = 0;

    motor2.PID_velocity.P = 0.05;
    motor2.PID_velocity.I = 1;
    motor2.PID_velocity.D = 0;

    // 驱动器设置
    motor1.voltage_sensor_align = 6;
    motor2.voltage_sensor_align = 6;
    driver1.voltage_power_supply = 8;
    driver2.voltage_power_supply = 8;
    driver1.init();
    driver2.init();

    //连接motor对象与驱动器对象
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::torque;
    motor2.controller = MotionControlType::torque;

    // monitor相关设置
    motor1.useMonitoring(Serial);
    motor2.useMonitoring(Serial);
    //电机初始化
    motor1.init();
    motor1.initFOC();
    motor2.init();
    motor2.initFOC();

    delay(1000);   
    printf("\n[INFO] setup_mushibot() is done. \n");
}

void Mushibot::loop_mushibot() {
    bat_check();         //电压检测
    mpu6050.update();    //IMU数据更新

    speed_prev = get_speed();
    status_prev = get_status();

    //迭代计算FOC相电压
    motor1.loopFOC();
    motor2.loopFOC();

    //设置轮部电机输出
    motor1.move();
    motor2.move();
}

float Mushibot::get_speed()
{
    float robot_speed = (-0.5) * (motor1.shaft_velocity + motor2.shaft_velocity);
    Serial.printf("\n[INFO] speed: %.2f \n", robot_speed);
    return robot_speed;
}

JsonDocument Mushibot::get_status()
{
    // 参数json
    JsonDocument doc;
    String paramsJsonString;

    // 添加"chartsname"数组
    JsonArray chartsName = doc["chartsname"].to<JsonArray>(); 
    chartsName.add("speed");
    chartsName.add("angle");

    // 添加"speed"对象
    JsonObject speed = doc["speed"].to<JsonObject>();
    speed["robot_speed"] = String(get_speed());
    speed["motor1_shaft_velocity"] = String(motor1.shaft_velocity);
    speed["motor2_shaft_velocity"] = String(motor2.shaft_velocity);

    // 添加"angle"对象
    JsonObject angle = doc["angle"].to<JsonObject>(); 
    angle["motor1_shaft_angle"] = String(motor1.shaft_angle);
    angle["motor2_shaft_angle"] = String(motor2.shaft_angle);

    // Serial print out the content of the JsonDocument
    Serial.printf("\n[INFO] status: \n");
    serializeJson(doc, Serial); 
    Serial.println();

    return doc;
}