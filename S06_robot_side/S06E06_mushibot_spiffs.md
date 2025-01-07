# Anatomy of Mushibot's Embedded File System

## 1. Objectives

The original Mushibot system doesn't use flash file system, we added a SPIFFS file system to Mushibot. 

The reason that we wrote this blog dedicated to the topic of embedded file system for ESP32 is that it is errorr-prone, 
especially because of the complicated and multiple configuration combination. 

&nbsp;
## 2. SPIFFS that works

This section provides a SPIFFS based file system implementation that works well with the Mushibot which uses a ESP32-WROOM-32 board. 

[LittleFS outperforms SPIFFS](https://www.techrm.com/file-management-on-esp32-spiffs-and-littlefs-compared/), and very likely to replace SPIFFS. 

| Feature                   | SPIFFS                  | LittleFS                            |
| ------------------------- | ----------------------- | ----------------------------------- |
| Directory support         | No                      | Yes                                 |
| Crash Resilience          | Limited                 | High                                |
| Memory Efficiency         | Good                    | Great                               |
| Implementation Complexity | Low                     | Average                             |
| Fragmentation Management  | Lower                   | Greater                             |
| Typical Applications      | Configuration File, Log | Critical Data, Complex Applications |

However, we didn't make LittleFS work properly in the Mushibot system, and the reason is unknown yet. 
In the later section of this blog, we took a record of how we used LittleFS in the Mushibot system, and what bugs we encountered. 

We tooks the following steps to make SPIFFS in the Mushibot system, 

1. Configure `platformio.ini`,
2. Create a partition table,
3. Make an image of the files, and upload to the ESP32 board, using Platformio tools,
4. Write C++ programs to manage the uploaded files,
5. Upload the programs to ESP32 board, and run them.   

&nbsp;
### 2.1 Configure platformio.ini


&nbsp;
### 2.2 Create a partition table

&nbsp;
### 2.3 Make an image of the files, and upload to the ESP32 board

&nbsp;
### 2.4 Write C++ programs to manage the uploaded files

&nbsp;
### 2.5 Upload the programs to ESP32 board, and run them


&nbsp;
## 3. LittleFFS failed

Similar to the previous section, we took 5 steps to use `LittleFS` in the Mushibot system. 

 

