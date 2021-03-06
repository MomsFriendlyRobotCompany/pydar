

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>    // data type declarations
#include <stdio.h>
#include <fcntl.h>     // File Control Definitions
#include <termios.h>   // POSIX Terminal Control Definitions
#include <unistd.h>    // UNIX Standard Definitions
#include <errno.h>     // ERROR Number Definitions
#include <sys/ioctl.h> // define both TIOCMBIS and TIOCMBIC


class Serial {
public:
    Serial();
    ~Serial();
    void open(char* port, uint16_t speed);
    void close();
    void write(uint8_t* buffer, uint16_t size);
    // uint16_t read(uint8_t* buffer, uint32_t size);
    uint16_t read();  // reads into internal buffer
    int inWaiting();
    bool isOpen();

    inline void rts(bool val){
        rtsdtr(val, TIOCM_RTS);
    }

    inline void dtr(bool val){
        rtsdtr(val, TIOCM_DTR);
    }

    inline void flushInput(){
        tcflush (fd, TCIFLUSH);
    }

    inline void flushOutput(){
        tcflush (fd, TCOFLUSH);
    }

protected:
    int init(uint16_t speed);
    void rtsdtr(bool val, int pin);  // set/clear the RTS/DTR pin

    int fd;             // file descriptor
    char buffer[1024];  // serial buffer

};

#endif
