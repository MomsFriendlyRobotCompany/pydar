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
    stop();
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

void URGLidar::start(){
    lidar_thread = std::thread(&URGLidar::loop, this);
}


void URGLidar::stop(){
    this->is_shutdown = true;
    delay(100);
}

void URGLidar::decode(uint8_t* bytes){
    ;
}

bool URGLidar::sendCmd(URGLidar::Command cmd){
    uint8_t msg[3];
    switch (cmd){
        case URGLidar::SCAN:
            msg[1] = 0x60;
            break;
        case URGLidar::STOP:
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
        case URGLidar::INFO:
            /*
            When an external device sends a Get Device Information command to A4
            (A5 90), X4 will feedback the device's model, firmware version, and
            hardware version, and the device's factory serial number.

            [A5][5A][14][0][0][0][04][...content...]
            content = [0:model][1-2: FW][3:HW][4-19:SN]
            */
            //uint8_t ver[3] = {'V', 'V', '\n'};
            // s.write(ver, 3, 1);
            // delay(10);
            msg[0] = 'V';
            msg[1] = 'V';
            msg[2] = '\n';
            break;
        case URGLidar::HEALTH:
            /*
            When the external device sends the Get Device Health Status command
            (A5 91) to X4, X4 will feedback the device's status code.
            */
            msg[1] = 0x91;
            break;
        case URGLidar::RESTART:
            /*
            When the external device sends the Get Device command to A4 (A5 40),
            X4 will start a soft reboot and the system will restart. This command
            does not answer.
            */
            msg[1] = 0x40;
            break;
        // case URGLidar::SCIP:
        default:
            return false;
    }
    this->serial.write(msg, sizeof(msg), 1);
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
    this->sendCmd(URGLidar::RESTART);

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

void URGLidar::motor(bool val){
    // if (val) serial.rts(true);
    // else serial.rts(false);
    uint8_t laser_on[3] = {'B','M','\n'};
    uint8_t laser_off[3] = {'Q','T','\n'};
    if (val) serial.write(laser_on, 3, 1);
    else serial.write(laser_off, 3, 1);
    delay(10);

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
