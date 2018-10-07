#include <iostream>
#include <string>
#include "imp.h"

using std::cout;
using std::endl;
using std::string;


int main(void){
    cout << "start" << endl;

    string port = "/dev/tty.SLAB_USBtoUART";

    Serial s = Serial();
    s.open(port, 115200);  // ydlidar speed
    if (s.isOpen()) printf(">> opened\n");
    else printf(">> couldn't open\n");

    // spin test
    // s.dtr(true);
    // delay(2000);
    // s.dtr(false);

    // info
    uint8_t info[2] = {0xA5, 0x90};
    s.write2(info, 2, 1);

    uint8_t health[2] = {0xA5, 0x92};
    // s.write2(health, 2, 1);

    delay(1000);

    // cout << "input buffer: " << s.inWaiting() << endl;

    const int buf_size = 1024;

    uint8_t buffer[buf_size];
    memset(buffer, 0, buf_size);
    // for(int i=0; i<buf_size; ++i) printf ("0x%02X ", buffer[i]);
    int num = s.read2(buffer, buf_size, 5);
    if (num) cout << "buffer " << num << " " << std::hex << buffer << endl;
    else cout << "crap, noting" << endl;

    s.close();

    return 0;
}
