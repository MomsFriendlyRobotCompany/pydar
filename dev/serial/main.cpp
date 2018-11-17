#include <iostream>
#include <string>
#include "imp.h"

using std::cout;
using std::endl;
using std::string;


void printString(string msg, uint8_t* buf, const int len){
    printf ("%s[%d]: ", msg.c_str(), len);
    // zero padding 2 places, capital letters
    for (int i = 0; i < len; i++) printf ("%c", buf[i]);
    printf ("\n");
}


int main(void){
    cout << "start" << endl;

    string port = "/dev/tty.SLAB_USBtoUART";
    // string port = "/dev/tty.usbserial-AL034G2K";
    // string port = "/dev/tty.usbmodemFD131";

    Serial s = Serial();
    // s.open(port, 115200);  // ydlidar speed
    s.open(port, 128000);  // ydlidar speed
    if (s.isOpen()) printf(">> opened\n");
    else printf(">> couldn't open\n");

    // spin test
    // s.dtr(true);
    // delay(2000);
    // s.dtr(false);

// harley.ryan.sam.1

    // reset
    uint8_t restart[2] = {0xA5, 0x40};
    s.write(restart, 2, 1);
    delay(10);

    // info
    uint8_t info[2] = {0xA5, 0x90};
    s.write(info, 2, 1);
    delay(10);

    // urg
    // uint8_t ver[3] = {'V', 'V', '\n'};
    // s.write(ver, 3, 1);
    // delay(10);

    // uint8_t health[2] = {0xA5, 0x91};
    // s.write2(health, 2, 1);
    // delay(10);

    // cout << "input buffer: " << s.inWaiting() << endl;

    const int buf_size = 1024;

    uint8_t buffer[buf_size];
    memset(buffer, 0, buf_size);
    // for(int i=0; i<buf_size; ++i) printf ("0x%02X ", buffer[i]);
    int num = s.read(buffer, buf_size, 5);
    if (num){
        printHex("response hex", buffer, num);
        printString("response string", buffer, num);
    }
    else cout << "crap, noting" << endl;

    s.close();

    return 0;
}
