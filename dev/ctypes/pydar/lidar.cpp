
#include <lidar.h>
#include <thread>
#include <chrono>  // time

// https://rafalcieslak.wordpress.com/2014/05/16/c11-stdthreads-managed-by-a-designated-class/


YDLidar::YDLidar(char* port){
    this->port = port;
    this->is_shutdown = false;
    this->baudrate = 9600;
}

YDLidar::~YDLidar(){
    // this->is_shutdown = true;
    // motor(false);
    // close();
    stop();
}

void YDLidar::start(){
    lidar_thread = std::thread(&YDLidar::loop, this)
}


void YDLidar::stop(){
    this->is_shutdown = true;
    usleep(1000);
}

void YDLidar::decode(uint8_t* bytes){
    ;
}

void YDLidar::loop(){
    serial = Serial();
    serial.open(this->port, this->baudrate);

    this->motor(true);

    // reset lidar
    uint8_t cmd[2] = {0x00, 0x00};
    serial.write(cmd, 2);
    serial.flushInput();

    // start scanning
    uint8_t cmd[2] = {0x00, 0x00};
    serial.write(cmd, 2);

    while (!this->is_shutdown){
        data = serial.read();
        // decode data
        // put data into this->buffer
        // usleep(1000);
    }

    this->motor(false);
    serial.close();
}

// void YDLidar::open(char* port){
//     serial = Serial();
//     serial.open(port, 9600);
// }
//
// void YDLidar::close(){
//     if (serial.isOpen()){
//         serial.close();
//     }
// }

void YDLidar::motor(bool val){
    if (val) serial.rts(true);
    else serial.rts(false);
    usleep(1000);
}

void YDLidar::get(ScanData* data){
    // just copies the buffer into the ScanData
    for (int i = 0; i < 2*360; ++i){
        data->data[i] = this->buffer[i];
    }
}

// char* YDLidar::info(){
//     uint8_t cmd[2] = {0x00, 0x00};
//     serial.write(cmd)
// }
//
// void YDLidar::reset(){
//     uint8_t cmd[2] = {0x00, 0x00};
//     serial.write(cmd)
// }
