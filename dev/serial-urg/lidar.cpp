#include "lidar.h"
#include <thread>
#include <mutex>
#include <chrono>  // time
#include <math.h>

// https://rafalcieslak.wordpress.com/2014/05/16/c11-stdthreads-managed-by-a-designated-class/


using std::string;
using std::cout;
using std::endl;
using std::lock_guard;
using std::thread;
using std::mutex;


std::thread lidar_thread;
std::mutex data_mutex;


// enum Commands {
//     INFO=0,
//     START_LASER,
//     STOP_LASER,
//     PARAMS
// };

// uint8_t info[3] = {'V', 'V', '\n'};

uint8_t CommandStrings[4][3] = {
    {'V', 'V', '\n'},  // info
    {'B', 'M', '\n'},  // laser on
    {'Q', 'T', '\n'},  // laser off
    {'P', 'P', '\n'}   // get parameters
};

// Convert FSA and LSA to an angular degree
inline double angle(uint8_t l, uint8_t h){ return (((h >> 1) << 8) + l) / 64.0; }

double angle_correction(double distance){
    double corr = 0.0;
    if (distance > 0) corr = 180/M_PI*atan(21.8*(155.3-distance)/(155.3*distance));
    return corr;
}


URGLidar::URGLidar(){
    // this->port = port;
    this->is_shutdown = false;
    // this->baudrate = 115200;
    // this->serial = Serial();
}

URGLidar::~URGLidar(){
    // this->is_shutdown = true;
    // motor(false);
    // close();
    stopThread();
}

bool URGLidar::init(string port){
    this->serial.open(port, 115200);
    bool ret = false;
    if(this->serial.isOpen()) ret = true;
    else return false;

    uint8_t cmd[] = "SCIP2.0\n";
    memset(this->buffer, 0, sizeof(this->buffer));
    int num = serial.read(buffer, sizeof(buffer), 2);
    return ret;
}

void URGLidar::startThread(){
    lidar_thread = std::thread(&URGLidar::loop, this);
}


void URGLidar::stopThread(){
    this->is_shutdown = true;
    delay(100);
}

int URGLidar::decode(uint8_t* bytes, uint32_t size){
    int decode = 0;
    for(int i=0; i<size; ++i){
        decode <<= 6;
        decode &= ~0x3f;
        decode |= bytes[i] - 0x30;
    }
    return decode;
}

double URGLidar::index2rad(int index){
    return (2.0*M_PI) * (index - this->AFRT) / this->ARES;
}

double URGLidar::scan_sec(){
    return 60.0/this->SCAN;
}

bool URGLidar::sendCmd(URGLidar::Command cmd){
    this->serial.write(CommandStrings[cmd], 3, 1);
    delay(1);
    return true;
}

void URGLidar::loop(){
    // serial = Serial();
    // this->serial.open(this->port, this->baudrate);

    this->motor(true);

    // reset lidar
    // uint8_t cmd[2] = {0x00, 0x00};
    // serial.write(cmd, 2);
    // this->restart();
    // this->sendCmd(URGLidar::RESTART);

    this->serial.flushInput();

    // start scanning
    // uint8_t go[2] = {0xA5, 0x60};
    // serial.write(go, 2);

    while (!this->is_shutdown){
        // https://github.com/aldebaran/liburg/blob/master/src/cpp/urg/CaptureSettings.h
        char buffer[] = "GDbbbbeeeegg\n";
        // snprintf(buffer, strlen(buffer) + 1, "GD%04d%04d%02u\n", begin, end, skiplines);



        // data = this->serial.read();
        // decode data
        // put data into this->buffer
        // usleep(1000);
    }

    this->motor(false);
    this->serial.close();
}

void URGLidar::motor(bool val){
    Command c;
    if (val) c = URGLidar::START_LASER;
    else c = URGLidar::STOP_LASER;
    this->sendCmd(c);

    // uint8_t buffer[64];
    int num = serial.read(buffer, sizeof(buffer), 3);
    // printf("motor[%d]: %s\n", num, buffer);
    printHex("motor", buffer, num);

    if(val){
        if( buffer[0] == 'B' && buffer[1] == 'M' && num == 8){
            printf(">> success on\n");
        }
        else printf("** on ret[%d]: %s **\n", num, buffer);
    }
    else {
        if( buffer[0] == 'Q' && buffer[1] == 'T' && num == 8){
            printf(">> success off\n");
        }
        else printf("** off ret[%d]: %s **\n", num, buffer);

    }

    this->serial.flushInput();
}

void URGLidar::get(ScanData* data){
    // just copies the buffer into the ScanData
    lock_guard<mutex> guard(data_mutex);
    for (int i = 0; i < 2*360; ++i){
        data->data[i] = this->scan_raw[i];
    }
}


bool URGLidar::get_param(){
    uint8_t cmd[] = "PP\n";
    serial.write(cmd, 3, 1);
    delay(100);
    uint8_t buffer[32];
    int num = serial.read(buffer, sizeof(buffer), 3);
    printf("[%d]: %s\n", num, buffer);

    return true;
}






// string URGLidar::info(void){
//     uint8_t cmd[2] = {0xA5, 0x90};
//     serial.write(cmd, 2);
// }
//
// void URGLidar::restart(){
//     uint8_t cmd[2] = {0xA5, 0x40};
//     serial.write(cmd, 2);
// }
//
// void URGLidar::scan(void){
//     uint8_t cmd[2] = {0xA5, 0x60};
//     serial.write(cmd, 2);
// }
