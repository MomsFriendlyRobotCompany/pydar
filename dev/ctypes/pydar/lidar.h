

#ifndef __LIDAR_H__
#define __LIDAR_H__


#include <imp.h>  // serial class


typedef struct ScanData{
    int len;
    double *data;
} ScanData;

class YDLidar {
    YDLidar();
    ~YDLidar();
    void start();
    // void open(char* port);
    // void close();
    void motor(bool val);
    void get(ScanData* data);
    // char* info();
    // void reset();
protected:
    void loop();

    bool is_shutdown;
    char* port;
    int baudrate;
    uint8_t buffer[360*2];
    std::thread lidar_thread;
};

#endif
