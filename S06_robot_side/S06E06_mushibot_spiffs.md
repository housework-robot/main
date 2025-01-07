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

This [`platformio.ini`](https://github.com/housework-robot/main/blob/main/S06_robot_side/S06E06_src/src/Mushibot20250107/platformio.ini) works well. 

It contains quite some lines that were commented, so as to be disfuntioned. We have tried those commented lines, and they behaved either unexpected or unnecessary. 

Following lines in the `platformio.ini` are related to SPIFFS. 

~~~
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
board_build.filesytem = spiffs
board_build.partitions = default_4MB.csv
;board_build.flash_mode = dio
;board_build.f_flash = 80000000L
~~~

* **env**, **platform** and **board**
  
  The Mushibot system uses a ESP32-WROOM-32 board.

  [The platformio official guide](https://docs.platformio.org/en/latest/boards/index.html)
  does provide many board options, but no one fits `ESP32-WROOM-32 board` well.

  After quite some trial and error, we found the follow configuration values worked well.

  ~~~
  [env:esp32dev]
  platform = espressif32
  board = esp32dev
  framework = arduino
  ~~~

  You can replace `platform = espressif32` with `platform = https://github.com/pioarduino/platform-espressif32/releases/download/51.03.04/platform-espressif32.zip`.

  But it doesn't work when using `platform = https://github.com/platformio/platform-espressif32.git`. 

* **board_build.filesytem**

  Referring to [the platformio official guide](https://docs.platformio.org/en/latest/platforms/espressif32.html#uploading-files-to-file-system),
  possible values for `board_build.filesystem` are `spiffs` (default), `littlefs` and `fatfs`.

  `littlefs` didn't work well in the Mushibot system, and we will discuss the bug in later section of this blog.

  So far, only `spiffs` works well for the Mushibot system.
  
* **board_build.partitions**

  We copied a partition table from the platformio official example repo, and pasted it as `default_4MB.csv`.

* **board_build.flash_mode** and **board_build.f_flash**

  Referring to [the platformio's official guide], we used the default values for these two parameters, and they worked well.

  ~~~
  ;board_build.flash_mode = dio
  ;board_build.f_flash = 80000000L
  ~~~

  `flash_mode` has 4 valid values, `qio`, `qout`, `dio` and `dout`. We tried `dio`, it behaved as expected.

  `f_flash` has 2 valid values, 80000000L for 80MHz frequency, and the default value is 40000000L for 40MHz.
  

&nbsp;
### 2.2 Create a partition table

We adopted [an official example partition table](https://github.com/espressif/arduino-esp32/blob/master/tools/partitions/default.csv), 
and renamed it to `default_4MB.csv`.  Following is its content,

| # Name  	| Type	 | SubType 	| Offset  	| Size    	| Flags |
|----------|-------|----------|----------|----------|-------|
| nvs     	| data 	| nvs      |	0x9000   |	0x5000   |      	|
| otadata	 | data	 | ota	     | 0xe000  	| 0x2000	  |       |
| app0    	| app  	| ota_0   	| 0x10000	 | 0x140000	|       |
| app1	    | app	  | ota_1   	| 0x150000	| 0x140000	|       |
| spiffs	  | data  | spiffs  	| 0x290000	| 0x160000 |	      |
| coredump	| data	 | coredump |	0x3F0000	| 0x10000  |	      |

Referring to [the official guide of partition table](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/partition-tables.html#subtype),
the `subtype` can be either `spiffs`, or `littlefs`, or `fat`.

We tried `littlefs`, and it failed. When using `spiffs`, it worked well.


&nbsp;
### 2.3 Make an image of the files,end upload to the ESP32 board

&nbsp;
### 2.4 Write C++ programs to manage the uploaded files

&nbsp;
### 2.5 Upload the programs to ESP32 board, and run them


&nbsp;
## 3. LittleFFS failed

Similar to the previous section, we took 5 steps to use `LittleFS` in the Mushibot system. 

 

