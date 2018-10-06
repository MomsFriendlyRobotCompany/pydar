
#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <iostream>
#include <thread>

#include <math.h>

inline double deg2rad(double d){ return d*M_PI/180.0; }
inline double rad2deg(double r){ return r*180.0/M_PI; }

/*
unsigned int ranges:
2**16 = 65536
2**32 = 4294967296
2**64 = 18446744073709551616
*/

#ifndef __cplusplus
#error "This requires a C++ compiler"
#endif


class Lidar {
public:
    Lidar();
    ~Lidar();
    void start(std::string port);
    void stop(void);
    string info();

protected:
    void loop(void);

    uint8_t raw[1024];
    double scan_raw[360*2];
    std::thread lidar_thread;
};

#endif
