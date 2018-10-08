
#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <iostream>
#include <thread>
#include "imp.h"
#include <math.h>

inline double deg2rad(double d){ return d*M_PI/180.0; }
inline double rad2deg(double r){ return r*180.0/M_PI; }

/*
unsigned int ranges:
2**16 = 65536
2**32 = 4294967296
2**64 = 18446744073709551616
*/

/*
use case

lidar = Lidar()
lidar.start("/dev/serial0") // create thread
while(run){
    lidar.get()
}
lidar.stop()
*/

#ifndef __cplusplus
#error "This requires a C++ compiler"
#endif

typedef struct ScanData{
    int len;
    double *data;
} ScanData;


class URGLidar {
public:
    // commands for lidar
    enum Command {
        SCAN,
        STOP,
        INFO,
        HEALTH,
        RESTART
    };

    URGLidar();
    ~URGLidar();
    bool init(std::string port);
    void start(void);              // start serial thread
    void stop(void);               // stop serial thread
    bool sendCmd(Command cmd);     // send command to lidar
    void motor(bool val);          // turn motor on/off
    void get(ScanData* data);      // get scan data
    // string info(void);
    // void restart(void);
    // string info();
    bool get_param();

protected:
    void loop(void);              // thread loop
    void decode(uint8_t* bytes);  // decode raw data into scan data

    uint8_t raw[1024];       // raw data buffer
    double scan_raw[360*2];  // scan data
    // std::thread lidar_thread;
    // std::mutex data_mutex;
    Serial serial;
    bool is_shutdown;
    std::string port;
    uint32_t baudrate;
    uint8_t buffer[64];
};

#endif
