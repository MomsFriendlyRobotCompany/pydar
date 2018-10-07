// https://www.cmrr.umn.edu/~strupp/serial.html
#include <stdio.h>     // printf
#include <fcntl.h>     // File Control Definitions
#include <termios.h>   // POSIX Terminal Control Definitions
#include <unistd.h>    // UNIX Standard Definitions
#include <errno.h>     // ERROR Number Definitions
#include <sys/ioctl.h> // define both TIOCMBIS and TIOCMBIC
#include <cstring>


#include <sys/param.h>
// #include <sys/signal.h>
// #include <errno.h>
//
// #include <sys/select.h>
// #include <sys/time.h>
// #include <time.h>
// #ifdef __MACH__
// #include <AvailabilityMacros.h>
// #include <mach/clock.h>
// #include <mach/mach.h>
// #endif

#include "imp.h"

#if __APPLE__
#include <IOKit/serial/ioss.h>
#endif

#include <iostream>

using std::string;
using std::cout;
using std::endl;

// http://web.mit.edu/cassa/rdesktop-1.5.0/serial.c
#ifndef CRTSCTS
    #define CRTSCTS 0
#endif
#ifndef CNEW_RTSCTS
    // #define CRTSCTS CNEW_RTSCTS
    #define CNEW_RTSCTS CRTSCTS
#endif

/* FIONREAD should really do the same thing as TIOCINQ, where it is
 * not available */
#if !defined(TIOCINQ) && defined(FIONREAD)
    #define TIOCINQ FIONREAD
#endif
#if !defined(TIOCOUTQ) && defined(FIONWRITE)
    #define TIOCOUTQ FIONWRITE
#endif

void printHex(string msg, uint8_t* buf, const int len){
    printf ("%s[%d]: ", msg.c_str(), len);
    // zero padding 2 places, capital letters
    for (int i = 0; i < len; i++) printf ("0x%02X ", buf[i]);
    printf ("\n");
}

void delay(uint16_t ms){
    // usleep arg should be < 1M
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    }
    usleep(ms*1000);
}

Serial::Serial(){
    fd = 0;

    #if __APPLE__
    printf("macOS\n");
    #elif __linux__
    printf("linux\n");
    #endif
}

Serial::~Serial(){
    close();
}

void Serial::open(string port, uint32_t speed){
    // fd = ::open(port.c_str(),
    //     O_RDWR |
    //     O_NOCTTY |
    //     O_NDELAY
    //     // O_NONBLOCK |
    //     // O_APPEND
    // ); // | O_NDELAY

    fd = ::open (port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND | O_NDELAY);

    printf("************************\n");
    if (fd <= 0 ){
        printf("serial: %s\n", port.c_str());
        printf("Couldn't open the port\n");
        printf("************************\n");
        return;
    }
    // cout << "fd: " << fd << endl;
    printf("serial: %s\n", port.c_str());
    printf("fd: %d\n", fd);
    printf("port is OPEN\n");
    printf("************************\n");

    fcntl(fd, F_SETFL, FNDELAY); // non-blocking mode?

    struct termios options;

    // get config from fd and put into options
    if (tcgetattr(fd, &options) == -1){
        printf("couldn't get port options");
        return;
    }

    // cout << options << endl;

    // Enable the receiver and set local mode
    // options.c_cflag |= (CLOCAL | CREAD);

    // set up raw mode / no echo / binary
    // options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
    // options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN); //|ECHOPRT
    //
    // options.c_oflag &= (tcflag_t) ~(OPOST);
    // options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

    //give raw data path
    // cfmakeraw (&options);
    // options.c_cflag &= (unsigned long) ~(CRTSCTS);
    //
    // // 8N1
    // options.c_cflag &= ~CSIZE;
    // options.c_cflag |= CS8;
    // options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
    // options.c_cflag &= (tcflag_t) ~(CSTOPB);
    //
    // options.c_lflag &= ~(ICANON|ISIG|IEXTEN|ECHO);
    // options.c_iflag &= ~(ISTRIP|ICRNL);
    // options.c_oflag &= ~OPOST;

    // // 8N1
    // options.c_cflag &= ~PARENB;
    // options.c_cflag &= ~CSTOPB;
    // options.c_cflag &= ~CSIZE;
    // options.c_cflag |= CS8;
    // no flow control
    // options.c_cflag &= ~CRTSCTS;
    //
    // //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
    //
    // options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    // options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    //
    // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    // options.c_oflag &= ~OPOST; // make raw

    // options.c_cflag |= ( CLOCAL | CREAD |  CS8);    // Configure the device : 8 bits, no parity, no control
    // options.c_iflag |= ( IGNPAR | IGNBRK );

    options.c_cflag |= ( CLOCAL | CREAD );

    // Set the Charactor size
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8;    /* Select 8 data bits */

    // Set parity - No Parity (8N1)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Disable Software Flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Chose raw (not processed) output
    options.c_oflag &= ~OPOST;

    // no blocking
    options.c_cc[VMIN]  = 0;    // no minimum number of chars to read
    options.c_cc[VTIME] = 1;    // 5: 0.5 seconds read timeout

    // set custom baud
    #if defined(__APPLE__)
        speed = 128000;
        speed_t new_baud = static_cast<speed_t> (speed);
        if(-1 == ioctl (fd, IOSSIOSPEED, &new_baud, 1)){
            printf("Error setting speed: %lu", new_baud);
        }
    #elif defined(__linux__)
        // cfsetspeed(&options, 128000);
        struct serial_struct sers;
        if (-1 == ioctl (fd, TIOCGSERIAL, &sers)){
            printf("Error getting serial ioctl");
        }

        // configure
        sers.custom_divisor = sers.baud_base / static_cast<int> (speed);
        sers.flags &= ~ASYNC_SPD_MASK;
        sers.flags |= ASYNC_SPD_CUST;

        // set serial speed
        if (-1 == ioctl (fd, TIOCSSERIAL, &sers)){
            printf("Error setting serial ioctl");
        }
    #else
        ::cfsetspeed(&options, B128000);
    #endif
    // ::cfsetispeed(&options, B128000);
    // ::cfsetospeed(&options, B128000);
    #ifdef B128000
    if (speed == 128000){
        printf("B128000 is defined\n");
        ::cfsetispeed(&options, B128000);
        ::cfsetospeed(&options, B128000);
    }
    #endif
    // ::cfsetspeed(&options, B115200);

    tcflush(fd,TCIFLUSH);
    fcntl(fd, F_SETFL, FNDELAY);
    //send options back to fd
    ::tcsetattr(fd, TCSANOW, &options);
    // ::tcsetattr(fd, TCSAFLUSH, &options);

    // init(speed);
    // printf("opened\n");
}

void Serial::close(){
    if(fd) ::close(fd);
    fd = 0;
}

int Serial::write(uint8_t* buf, const int numbytes, const int trys){
    int i, numwritten = 0, n = 0, numzeroes = 0;

    // cout << "msg: " << "  " << std::hex << int(buf[0]) << endl;
    // printf("hello\n");

    while (numwritten < numbytes){
        n = ::write (fd, (buf + numwritten), (numbytes - numwritten));

        if (n < 0){
            printf("write error: %d\n", n);
            return -1;
        }
        else if (0 == n){
            numzeroes++;

            if (numzeroes > trys) break;
        }
        else numwritten += n;
    }

    if (true) printHex("write", buf, numwritten);

    return numwritten;
}

// uint16_t Serial::read(){
//     uint16_t ret = 0;
//     ssize_t num = ::read(fd, buffer, sizeof(buffer));  // ssize_t int16_t +/-
//     if (num > 0) ret = (uint16_t) num;
//     return ret;
// }

int Serial::read(uint8_t* buf, const uint32_t numbytes, const uint8_t trys){
    int i;
    uint32_t numread = 0;
    int n = 0;
    uint8_t numzeroes = 0;

    // int t1 = 0;
    // ioctl(fd, FIONREAD, &t1);
    // printf("to read: %d\n", t1);

    while (numread < numbytes){
        //printf("%d .\n", fd);
        n = ::read (fd, (buf + numread), (numbytes - numread));

        // printf("%d\n", n);
        // printHex("debug", buf, 8);

        if (n < 0){
            printf("fd: %d\n", fd);
            printf("read error\n");
            return -1;
        }
        else if (0 == n){
            numzeroes++;
            if (numzeroes > trys){
                // printf("read: too many tryes: %d\n", numzeroes);
                break;
            }
            // printf(".");
            delay(500);
        }
        else numread += n;
    }

    // printf("\n");

    if (true) printHex("read", buf, numread);

    //tcflush (fd, TCIFLUSH);			//discard data that was not read
    return numread;
}

// can do the same for DTR: TIOCM_DTR
// void Serial::rts(bool val){
//     int RTS_flag = TIOCM_RTS;       // defined in termios.h
//     if (val) ioctl(fd, TIOCMBIS, &RTS_flag); // Set RTS pin
//     else ioctl(fd, TIOCMBIC, &RTS_flag); // Clear RTS pin
// }

void Serial::rtsdtr(bool val, int pin){
    int flag = pin;       // defined in termios.h
    if (val) ioctl(fd, TIOCMBIS, &flag); // Set RTS pin
    else ioctl(fd, TIOCMBIC, &flag); // Clear RTS pin
}

int Serial::inWaiting(){
    int num = 0;
    int err = ioctl(fd, TIOCINQ, &num);  // -1 if error
    if (err == -1) num = 0;
    return num;
}

bool Serial::isOpen(){
    bool ret = false;
    if (fd > 0) ret = true;
    return ret;
}
