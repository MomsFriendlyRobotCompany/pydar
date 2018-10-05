
#include <stdio.h>
#include <fcntl.h>     // File Control Definitions
#include <termios.h>   // POSIX Terminal Control Definitions
#include <unistd.h>    // UNIX Standard Definitions
#include <errno.h>     // ERROR Number Definitions
#include <sys/ioctl.h> // define both TIOCMBIS and TIOCMBIC


Serial::Serial(){
    fd = NULL;
}

Serial::~Serial(){
    close();
}

void Serial::open(char* port, uint16_t speed){
    fd = ::open(port, O_RDWR | O_NOCTTY);
    init(speed);
}

void Serial::close(){
    if(fd) ::close(fd);
    fd = NULL;
}

int Serial::init(uint16_t speed){
  int err = 0;
  struct termios options;
  memset (&tty, 0, sizeof tty);  // clear it

  // grab current settings
  err = tcgetattr(fd, &options);
  if (err != 0){
    printf("error reading port options");
    return 1;
  }

  // set input/output speeds
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  // no blocking
  options.c_cc[VMIN]  = 0;    // no minimum number of chars to read
  options.c_cc[VTIME] = 5;    // 0.5 seconds read timeout

  options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  // 8N1
  options.c_cflag &= ~PARENB
  options.c_cflag &= ~CSTOPB
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  options.c_iflag &= ~IGNBRK;         // disable break processing
  options.c_lflag = 0;                // no signaling chars, no echo,
                                      // no canonical processing
  options.c_oflag = 0;                // no remapping, no delays

  // set HW flow control
  // options.c_cflag |= CNEW_RTSCTS;

  // clear HW flow control
  options.c_cflag &= ~CNEW_RTSCTS;

  // set port options
  err = tcsetattr(fd, TCSANOW, &options);
  if (err != 0){
    printf("error setting up port");
    return 1;
  }

  return 0;
}

void Serial::write(uint8_t* buffer, uint16_t size){
    ::write(fd, data, sizeof(data));
    usleep(1000);
}

uint16_t Serial::read(){
    uint16_t ret = 0;
    ssize_t num = ::read(fd, buffer, sizeof(buffer));  // ssize_t int16_t +/-
    if (num > 0) ret = (uint16_t) num;
    return ret;
}

// can do the same for DTR: TIOCM_DTR
// void Serial::rts(bool val){
//     int RTS_flag = TIOCM_RTS;       // defined in termios.h
//     if (val) ioctl(fd, TIOCMBIS, &RTS_flag); // Set RTS pin
//     else ioctl(fd, TIOCMBIC, &RTS_flag); // Clear RTS pin
// }

void Serial::rtsdtr(bool val, pin){
    int flag = pin;       // defined in termios.h
    if (val) ioctl(fd, TIOCMBIS, &flag); // Set RTS pin
    else ioctl(fd, TIOCMBIC, &flag); // Clear RTS pin
}

int Serial::inWaiting(){
    int num = 0;
    err = ioctl(fd, TIOCINQ, &num));  // -1 if error
    if (err == -1) num = 0;
    return num;
}

bool isOpen(){
    bool ret = false;
    if (fd > 0) ret = true;
    return ret;
}
