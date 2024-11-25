# Serial Communication between Arduino and Python

              by Kan Deng, Nov 25, 2024

-------------------

## 1. Objectives

We want to integrate [D-Robotics' RDK-X5 board](https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x5) 
with [DengFOC driver board](https://github.com/ToanTech/Deng-s-foc-controller) for Brushless DC Electric Motors (BLDC) 
in [a simple balancing bot](https://github.com/ToanTech/Balance_Bot_DengFOC). 

   <p align="center">
     <img alt="the frontside of the balancing bot" src="./S06E02_src/balancing_bot_frontside.jpg" width="48%">
     &nbsp;  
     <img alt="the backside of the balancing bot" src="./S06E02_src/balancing_bot_backside.jpg" width="48%">
   </p>

In more detail, 

1. The balancing bot has two 2204 brushless motors, two AS5600 motor sensors, one MPU6050 IMU sensor.
   These motors and sensors are controlled by a DengFOC driver board, which is installed inside the white box.

3. [A ESP32 (wemos lolin32 lite) module](https://mischianti.org/esp32-wemos-lolin32-lite-high-resolution-pinout-and-specs/) is connected to the DengFOC driver board.

    The ESP32 takes charge of running Arduino codes, control the DengFOC driver, and communicating with a RDK-X5 board.
    It exposes its USB port on the backside of the balancing bot, and its pins are plugging into the DengFOC driver board inside the white box. 

4. A RDK-X5 board is assembled on the frontside of the balancing bot, and is connected to a RDK binocular camera on the top of the bot. 

5. RDK-X5 board behaves like an organizer, it receives the video stream captured by the RDK's camera,
   and pushes to the remote [SRS streaming server](https://ossrs.io/lts/en-us/docs/v6/doc/getting-started).

6. RDK-X5 uses AI large model to analyze the video stream, and recognize the environment for navigation.

   RDK-X5 receives the speeds of the motors, the pose of the balancing bot, from the ESP32 module and DengFOC driver board.
   
   Taking the environmental analysis result and the motion status of the balancing bot as inputs,
   the RDK-X5 runs a policy model to output the command, which usually consists of speed, steering angle etc. 

In order to empower the RDK-X5 to receive motion status from the ESP32, and send command back to them, 
we need to implement a communication channel between the RDK-X5 board and the ESP32 module. 

Since both the RDK-X5 board and the ESP32 module have wifi and bluetooth, wireless communication is one option. 

Another option is to use cable, especially for serial communication. 

Look into the pinouts of [the ESP32 wemos lolin32 lite module](https://mischianti.org/esp32-wemos-lolin32-lite-high-resolution-pinout-and-specs/) and [the RDK-X5 board](https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_Product_Brief_V1.0.pdf), UART, I2C, and SPI are all candidate solutions. 

   <p align="center">
     <img alt="the pinouts of ESP32 wemos lolin32 lite module" src="./S06E02_src/ESP32-WeMos-LOLIN32-Lite-pinout-mischianti.png" width="48%">
     &nbsp;  
     <img alt="the pinouts of RDK-X5 board" src="./S06E02_src/RDK-X5-pinout.png" width="48%">
   </p>

In this article, we implemented UART serial communication, sending and receiving JSON messages. 


&nbsp;
## 2. Hardware assembly

Take a look inside the balancing bot, where the DengFOC driver board is installed. 
The DengFOC driver board is used as a shield of the ESP32 module, 
so that all the pins of ESP32 module are plugged into DengFOC driver board, and not available for other usage. 

   <p align="center">
     <img alt="the DengFOC driver board inside the white box" src="./S06E02_src/DengFOC_inside.jpg" width="48%">
     &nbsp;  
     <img alt="the pins of DengFOC driver are used for ESP32 shield" src="./S06E02_src/DengFOC_pins.jpg" width="48%">
   </p>

Now that we cannot connect the RDK-X5 board to ESP32 module, how about connecting RDK-X5 to the DengFOC driver board? 
The DengFOC driver board has a few pins for external communication, 
but they are used for [the I2C connection for the AS5600 motor sensors](https://github.com/ToanTech/Balance_Bot_DengFOC). 
Therefore, it is not feasible to connect the RDK-X5 board to DengFOC driver board. 

   <p align="center">
     <img alt="the DengFOC driver board's pins for the I2C with the AS5600 motor sensors" src="./S06E02_src/DengFOC_driver_board.png" width="50%">
   </p>

If we want to use cable to connect the RDK-X5 board to the ESP32 module, 
and don't want to change the hardware structure of the balancing bot, the only candidate solution is to use USB cable. 

However, since there is only one USB port in the ESP32 module, it must be the `/dev/ttyUSB0`, 
which is usually reserved for testing purpose. 
What will happen if we use `/dev/ttyUSB0` for the serial communication, for production? 

&nbsp;
## 3. Source code

Suppose we use a USB cable to connect the RDK-X5 board with the ESP32 module, how can we send and receive JSON messages between them?

### 3.1 Related work

1. An engineer of the D-Robotics, the manufacturer of the RDK-X5 board,
   wrote [a blog on the serial communication between RDK-X3 and ESP32 module](https://blog.csdn.net/SA2672873269/article/details/136834623).

   He used RDK-X3's pin 8 and pin 10 for UART serial communication, corresponding to `/dev/ttyS3`.
   Although he successfully sent and received bytes between the RDK-X3 board and the ESP32 module,
   he didn't parse the bytes into data structures like `str` and `dict` etc.

2. There is [a popular tutorial on how to parse the bytes of serial communication](https://forum.arduino.cc/t/serial-input-basics-updated/382007).
   However, it is quite time-consuming to implement a serial communication parser from scratch.

3. [`SerialTransfer`](https://github.com/PowerBroker2/SerialTransfer) and [`pySerialTransfer`](https://github.com/PowerBroker2/pySerialTransfer) is sibling open projects
   to provide parsers for the serial communication on both Arduino side and python side.

   The libraries support multiple serial communcation, including UART, I2C and SPI.

   The libraries work quite well to send and receive simple messages, like float and string. But they doesn't behave correctly when exchanging JSON messages. 

4. [`ArduinoJson`](https://github.com/bblanchon/ArduinoJson) is an open source project,
   to provide powerful and easy-to-use library for JSON serialization and deserialization on Arduino board.

   One solution to work around the pitfall of `SerialTransfer`,
   is to use `ArduinoJson` to serialize JSON into string before sending message,
   and use `ArduinoJson` again to deserialize string into JSON when receiving message.
   

### 3.2 Source code

We upload our source codes to this repo, which consist of 2 tiers, 
one for Arduino running on ESP32 module, 
the other for Python which will run on RDK-X5 board, but for testing purpose, it runs in a ubuntu computer temporarily. 

#### 1. [The arduino tier](./S06E02_src/arduino_tier)

There are 3 sketches on the arduino tier.

1. `arduino_tier/arduino_tier.ino` is for the gateway, which runs the serial communication.

2. `arduino_tier/balancing_bot.h` and `arduino_tier/balancing_bot.cpp` for the control of the balancing bot, to make it moving and keep balanced. 

In the loop of `arduino_tier.ino`, 

1. First it checks if there is any JSON message from the python tier, by calling `receive_json()`. 

2. Once receiving the JSON message, it sends it back to the python tier like echoing, by calling `send_json(cmd)`. 

3. The arduino sketch collects the motion status of the balancing bot, including the 2 motor speeds, and the roll/pitch/yaw angles of the bot body, by calling `get_observation()`. 

4. It sends part of the observations to the python tier, by calling `send_json(obs)`.

5. After then, the arduino sketch makes decision of the next action by calling `policy(obs, cmd)`, and take step to control the motion of the bot by `step(action)`. 

6. The arduino code also writes some log information into the serial, using `Serial.printf(...)`. 

~~~
void loop() {
  // put your main code here, to run repeatedly:

  // remote command is received by the upper_tier from the remote server.
  JsonDocument cmd = receive_json();
  send_json(cmd);

  JsonDocument obs = blc_bot.get_observation();
  send_json(obs);
  JsonDocument action = blc_bot.policy(obs, cmd);

  /*
  Serial.printf("pitch_angle: %f, motor0_velocity: %f, motor1_velocity: %f, motor0_target: %f, motor1_target: %f \n", 
    obs["pitch_angle"], obs["motor0_velocity"], obs["motor1_velocity"], action["motor0_target"], action["motor1_target"]
  );
  */
  Serial.printf("%f %f %f %f %f\n", 
    obs["pitch_angle"], obs["motor0_velocity"], obs["motor1_velocity"], action["motor0_target"], action["motor1_target"]
  );
  blc_bot.step(action);
  // delay(500);
}
~~~

&nbsp;
#### 2. [The RDK tier](./S06E02_src/rdk_tier)

So far there is only 1 python script [`serial_channel.py`](./S06E02_src/rdk_tier/serial_channel.py). 
In the future, there will be more scripts added to the RDK tier. 

1. In the loop, the python script sends a JSON message to the arduino tier, by calling `send_json(send_data)`.

   To distinct the different loop, `cnt` increases 1 for every loop, `throttle` increases 0.11, and `steer` increases 0.22.  

2. Also in the loop, the python script receives the JSON messages from the arduino tier, by calling `receive_json()`.

   Notice that, since the arduino sketch sends two kinds of messages, one for `send_json(cmd)`, the other for `send_json(obs)`,
   the python script receives these two kinds of messages, in an interleaved manner. 
   
~~~
def testrun_serial():
    channel = SerialChannel()

    cnt = 0
    send_data = {
        "cnt": cnt, 
        "throttle": 0.11,
        "steer": 0.22
    }
    status = channel.send_json(send_data)

    while True:
        print(f"[{cnt}]")
        receive_data = channel.receive_json()
   
        if "ERROR" in receive_data:
            ...
        else:
            print(f"receive_data: {receive_data}")
        
        send_data["cnt"] = cnt
        send_data["throttle"] = send_data["throttle"] + 0.01
        send_data["steer"] = send_data["steer"] + 0.01

        channel.send_json(send_data)
        print(f"sendStr = '{send_data}' \n ")

        cnt += 1
~~~



&nbsp;
## 4. Run and results

1. In a computer, we startup an Arduino IDE, opening the arduino-tier's sketches.

   And then using a USB cable, connect the computer to the ESP32 module,
   load the arduino sketches, from the Arduino IDE to the ESP32 module installed on the balancing bot.

2. Push the button on the balancing bot, to start the DengFOC driver board and also the motors etc.

3. In the computer, open a CLI terminal, and run the python script,

   ~~~
   $ python3 serial_channel.py
   ~~~

Following image is a screen snapshot of the CLI terminal, displaying the running result of `serial_channel.py`. 

   <p align="center">
     <img alt="the running result of the python tier" src="./S06E02_src/python_tier_running_result.png" width="50%">
   </p>

The indices in the square brackets are the number of loops. 

It receives two kinds of messages from the arduino tier, one for command echo, the other for observation. 
This behavior is the correct one as expected. For example, 

~~~
...
[3980]
receive_data: {'pitch_angle': 29.80859, 'motor0_velocity': 0, 'motor1_velocity': 0}
sendStr = '{'cnt': 3980, 'throttle': 39.92000000000063, 'steer': 40.030000000000605}' 
...
[3988]
receive_data: {'cnt': 3980, 'throttle': 39.92, 'steer': 40.03}
sendStr = '{'cnt': 3988, 'throttle': 40.00000000000061, 'steer': 40.11000000000059}' 
...
~~~

Click the following image to view the video of the balancing bot. 

Obviously, the balancing bot didn't run smoothly, and it could keep balanced when running on ground. 

This unexpected behavior is caused by the two `send_json()` in the arduino sketch.
If deleting those `send_json()`, the balancing bot will be capable of moving straight on ground and keep balanced. 

This means that we cannot the USB port, as the serial communication channel between the RDK board and the ESP32 module. 

Therefore, we will change the hardware structure of the simple balancing bot, to empower it with RDK board and various peripheral like camera, to be capable of doing more complex tasks. 

   [![The balancing bot doesn't run smoothly because its USB port is occupied](https://img.youtube.com/vi/_8OrdSzhsWY/hqdefault.jpg)](https://www.youtube.com/watch?v=_8OrdSzhsWY)
   
