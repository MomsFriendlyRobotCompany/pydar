#include "lidar.h"
#include <thread>
#include <mutex>
#include <chrono>  // time
#include <math.h>

// https://rafalcieslak.wordpress.com/2014/05/16/c11-stdthreads-managed-by-a-designated-class/


using std::string;
using std::cout;
using std::endl;

// Convert FSA and LSA to an angular degree
inline double angle(uint8_t l, uint8_t h){ return (((h >> 1) << 8) + l) / 64.0; }

double angle_correction(double distance){
    double corr = 0.0;
    if (distance > 0) corr = 180/M_PI*atan(21.8*(155.3-distance)/(155.3*distance));
    return corr;
}


YDLidar::YDLidar(string port){
    this->port = port;
    this->is_shutdown = false;
    this->baudrate = 115200;
    // this->serial = Serial();
}

YDLidar::~YDLidar(){
    // this->is_shutdown = true;
    // motor(false);
    // close();
    stop();
}

void YDLidar::start(){
    this->lidar_thread = std::thread(&YDLidar::loop, this);
}


void YDLidar::stop(){
    this->is_shutdown = true;
    delay(100);
}

void YDLidar::decode(uint8_t* bytes){
    ;
}

bool YDLidar::sendCmd(YDLidar::Command cmd){
    uint8_t msg[2] = {0xA5, 0x00};
    switch (cmd){
        case YDLidar::SCAN:
            msg[1] = 0x60;
            break;
        case YDLidar::STOP:
            /*
            When the system is in the scanning state, X4 has been sending out point
            cloud data. If you need to close the scan at this time, you can send
            this command to stop the system from scanning. After sending the stop
            command, the system will be in standby state. At this point, the
            device's ranging unit is in low power mode and the laser is off.

            The command is unresponsive, so the system will not reply with any
            message after receiving the command.
            */
            msg[1] = 0x65;
            break;
        case YDLidar::INFO:
            /*
            When an external device sends a Get Device Information command to A4
            (A5 90), X4 will feedback the device's model, firmware version, and
            hardware version, and the device's factory serial number.

            [A5][5A][14][0][0][0][04][...content...]
            content = [0:model][1-2: FW][3:HW][4-19:SN]
            */
            msg[1] = 0x90;
            break;
        case YDLidar::HEALTH:
            /*
            When the external device sends the Get Device Health Status command
            (A5 91) to X4, X4 will feedback the device's status code.
            */
            msg[1] = 0x91;
            break;
        case YDLidar::RESTART:
            /*
            When the external device sends the Get Device command to A4 (A5 40),
            X4 will start a soft reboot and the system will restart. This command
            does not answer.
            */
            msg[1] = 0x40;
            break;
        default:
            return false;
    }
    this->serial.write(msg, 2, 1);
    delay(1);
    return true;
}

void YDLidar::loop(){
    // serial = Serial();
    this->serial.open(this->port, this->baudrate);

    this->motor(true);

    // reset lidar
    // uint8_t cmd[2] = {0x00, 0x00};
    // serial.write(cmd, 2);
    // this->restart();
    this->sendCmd(YDLidar::RESTART);

    this->serial.flushInput();

    // start scanning
    // uint8_t go[2] = {0xA5, 0x60};
    // serial.write(go, 2);

    while (!this->is_shutdown){
        // data = this->serial.read();
        // decode data
        // put data into this->buffer
        // usleep(1000);
    }

    this->motor(false);
    this->serial.close();
}

void YDLidar::motor(bool val){
    if (val) serial.rts(true);
    else serial.rts(false);
    delay(10);
}

void YDLidar::get(ScanData* data){
    // just copies the buffer into the ScanData
    std::lock_guard<std::mutex> guard(data_mutex);
    for (int i = 0; i < 2*360; ++i){
        data->data[i] = this->scan_raw[i];
    }
}





// string YDLidar::info(void){
//     uint8_t cmd[2] = {0xA5, 0x90};
//     serial.write(cmd, 2);
// }
//
// void YDLidar::restart(){
//     uint8_t cmd[2] = {0xA5, 0x40};
//     serial.write(cmd, 2);
// }
//
// void YDLidar::scan(void){
//     uint8_t cmd[2] = {0xA5, 0x60};
//     serial.write(cmd, 2);
// }
