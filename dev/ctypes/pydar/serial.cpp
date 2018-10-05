#include <stdio.h>
#include <fcntl.h>     // File Control Definitions
#include <termios.h>   // POSIX Terminal Control Definitions
#include <unistd.h>    // UNIX Standard Definitions
#include <errno.h>     // ERROR Number Definitions
#include <sys/ioctl.h> // define both TIOCMBIS and TIOCMBIC

/** References
https://xanthium.in/Serial-Port-Programming-on-Linux
http://xanthium.in/Controlling-RTS-and-DTR-pins-SerialPort-in-Linux
*/
//
// int init(int fd, int speed){
//   int err = 0;
//   struct termios options;
//   memset (&tty, 0, sizeof tty);  // clear it
//
//   // grab current settings
//   err = tcgetattr(fd, &options);
//   if (err != 0){
//     printf("error reading port options");
//     return 1;
//   }
//
//   // set input/output speeds
//   cfsetispeed(&options, speed);
//   cfsetospeed(&options, speed);
//
//   // no blocking
//   options.c_cc[VMIN]  = 0;    // no minimum number of chars to read
//   options.c_cc[VTIME] = 5;    // 0.5 seconds read timeout
//
//   options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
//
//   // 8N1
//   options.c_cflag &= ~PARENB
//   options.c_cflag &= ~CSTOPB
//   options.c_cflag &= ~CSIZE;
//   options.c_cflag |= CS8;
//
//   options.c_iflag &= ~IGNBRK;         // disable break processing
//   options.c_lflag = 0;                // no signaling chars, no echo,
//                                       // no canonical processing
//   options.c_oflag = 0;                // no remapping, no delays
//
//   // set HW flow control
//   // options.c_cflag |= CNEW_RTSCTS;
//
//   // clear HW flow control
//   options.c_cflag &= ~CNEW_RTSCTS;
//
//   // set port options
//   err = tcsetattr(fd, TCSANOW, &options);
//   if (err != 0){
//     printf("error setting up port");
//     return 1;
//   }
//
//   return 0;
// }

// can do the same for DTR: TIOCM_DTR
void setRTS(int fd){
  int RTS_flag = TIOCM_RTS;       // defined in termios.h
  ioctl(fd, TIOCMBIS, &RTS_flag); // Set RTS pin
}

void clearRTS(int fd){
  int RTS_flag = TIOCM_RTS;       // defined in termios.h
  ioctl(fd, TIOCMBIC, &RTS_flag); // Clear RTS pin
}

int serial_port(){
  int fd;

  // O_RDWR   - Read/Write access to serial port
  // O_NOCTTY - No terminal will control the process
  // O_SYNC - data is written synchronously
  // O_NDELAY - tells UNIX to ignore state of DCD signals
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

  if (fd == -1){
    printf("\n  Error! in Opening ttyUSB0\n\n");
    return 1;
  }
  else
    printf("\n\n  ttyUSB0 Opened Successfully\n\n");

  int err = init(fd, B115200);
  if (err != 0) return 1;

  // do stuff ------------------------
  write (fd, "hello!\n", 7);
  usleep ((7 + 25) * 100);
  char buf [100];
  int n = read (fd, buf, sizeof buf);

  // All done -------------------------
  close(fd);
  return 0;
}

//
// class Serial {
// public:
//     Serial(){
//         fd = NULL;
//     }
//     ~Serial(){
//         close();
//     }
//     void open(string port, int speed){
//         fd = ::open(port, O_RDWR | O_NOCTTY);
//         init(speed);
//     }
//
//     void close(){
//         if(fd) ::close(fd);
//         fd = NULL;
//     }
//
//
//     int init(int speed){
//       int err = 0;
//       struct termios options;
//       memset (&tty, 0, sizeof tty);  // clear it
//
//       // grab current settings
//       err = tcgetattr(fd, &options);
//       if (err != 0){
//         printf("error reading port options");
//         return 1;
//       }
//
//       // set input/output speeds
//       cfsetispeed(&options, speed);
//       cfsetospeed(&options, speed);
//
//       // no blocking
//       options.c_cc[VMIN]  = 0;    // no minimum number of chars to read
//       options.c_cc[VTIME] = 5;    // 0.5 seconds read timeout
//
//       options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
//
//       // 8N1
//       options.c_cflag &= ~PARENB
//       options.c_cflag &= ~CSTOPB
//       options.c_cflag &= ~CSIZE;
//       options.c_cflag |= CS8;
//
//       options.c_iflag &= ~IGNBRK;         // disable break processing
//       options.c_lflag = 0;                // no signaling chars, no echo,
//                                           // no canonical processing
//       options.c_oflag = 0;                // no remapping, no delays
//
//       // set HW flow control
//       // options.c_cflag |= CNEW_RTSCTS;
//
//       // clear HW flow control
//       options.c_cflag &= ~CNEW_RTSCTS;
//
//       // set port options
//       err = tcsetattr(fd, TCSANOW, &options);
//       if (err != 0){
//         printf("error setting up port");
//         return 1;
//       }
//
//       return 0;
//     }
//
//     void write(data){
//         ::write(fd, data, sizeof(data));
//         usleep(1000);
//     }
//
//     int read(){
//         int num = ::read(fd, buffer, sizeof(buffer));
//         return num;
//     }
//
//     // can do the same for DTR: TIOCM_DTR
//     // void rts(bool val){
//     //     int RTS_flag = TIOCM_RTS;       // defined in termios.h
//     //     if (val) ioctl(fd, TIOCMBIS, &RTS_flag); // Set RTS pin
//     //     else ioctl(fd, TIOCMBIC, &RTS_flag); // Clear RTS pin
//     // }
//
//     void rtsdtr(bool val, pin){
//         int flag = pin;       // defined in termios.h
//         if (val) ioctl(fd, TIOCMBIS, &flag); // Set RTS pin
//         else ioctl(fd, TIOCMBIC, &flag); // Clear RTS pin
//     }
//
//     inline void rts(bool val){
//         rtsdtr(val, TIOCM_RTS);
//     }
//
//     inline void dtr(bool val){
//         rtsdtr(val, TIOCM_DTR);
//     }
//
//     int inWaiting(){
//         int num = 0;
//         err = ioctl(fd, TIOCINQ, &num));  // -1 if error
//         if (err == -1) num = 0;
//         return num;
//     }
//
//     inline void flushInput(){
//         tcflush (fd, TCIFLUSH);
//     }
//
//     inline void flushOutput(){
//         tcflush (fd, TCOFLUSH);
//     }
//
//     int fd;             // file descriptor
//     char buffer[1024];  // serial buffer
//
// }
