/*
   Modified from:
   https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/examples/LITTLEFS_test/LITTLEFS_test.ino
*/

#ifndef EMBEDDED_FS_H_
#define EMBEDDED_FS_H_


#include <Arduino.h>
#include <FS.h>
// #include <SPIFFS.h>
#include <LittleFS.h>


class EmbeddedFS
{
public:
    EmbeddedFS();
    virtual ~EmbeddedFS();

    void setup_embeddedfs(); 
    void loop_embeddedfs();

    void list_dir(String dir_name, int level);
    void create_dir(String dir_path);
    void delete_dir(String dir_path);

    void read_file_to_serial(String file_dirname);
    void write_file(String file_dirname, String msg);
    void append_file(String file_dirname, String msg);
    void rename_file(String file_dirname1, String file_dirname2);
    void delete_file(String file_dirname);

private:
};

#endif
