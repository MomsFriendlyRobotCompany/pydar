#include <iostream>
#include <string>
#include "imp.h"

using std::cout;
using std::endl;
using std::string;


int main(void){
    std::cout<<"start"<<std::endl;

    string port = "/dev/tty.SLAB_USBtoUART";

    Serial s = Serial();
    s.open(port, 128000);  // ydlidar speed
    if (s.isOpen()) printf(">> opened\n");
    else printf(">> couldn't open\n");

    // spin test
    // s.dtr(true);
    // usleep(2000000);
    // s.dtr(false);

    // info
    uint8_t info[2] = {0xA5, 0x90};
    s.write2(info, 2, 1);

    uint8_t health[2] = {0xA5, 0x91};
    s.write2(health, 2, 1);

    usleep(1000000);

    cout << "input buffer: " << s.inWaiting() << endl;

    uint8_t buffer[1024] = {0};
    int num = s.read2(buffer, 1024, 5);
    if (num) cout << "buffer " << num << " " << std::hex << buffer << endl;
    else cout << "crap, noting" << endl;

    s.close();

    return 0;
}
