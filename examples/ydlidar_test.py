#!/usr/bin/env python3

from pydar.ydlidar import YDLidar
import time
import platform
# DTR - motor en

if __name__ == "__main__":
    if platform.system() == 'Linux':
        # port = "/dev/ttyUSB1"
        port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    elif platform.system() == 'Darwin':
        port = '/dev/tty.SLAB_USBtoUART'

    lidar = YDLidar()
    lidar.open(port, 128000)
    lidar.restart()

    lidar.start()

    # info = lidar.info()
    # print(info)

    # lidar.start_motor()

    # print(lidar.health())

    lidar.motor(True)
    time.sleep(1)
    ans = lidar.get()
    print('get returned:', ans)
    # time.sleep(1)
    # lidar.stop_motor()

    # time.sleep(1)
    # lidar.get()
    # lidar.get()
    # lidar.get()

    # lidar.serial.dtr = 1
    # info = lidar.info()
    # print(info)
    # lidar.serial.dtr = 0
    # info = lidar.info()
    # print(info)

    lidar.motor(False)
    time.sleep(1)
