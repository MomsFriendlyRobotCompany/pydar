#!/usr/bin/env python3

from pydar.ydlidar import YDLidar
import time
import platform
# DTR - motor en

if __name__ == "__main__":
    if platform.system() == 'Linux':
        port = "/dev/ttyUSB1"
    elif platform.system() == 'Darwin':
        port = '/dev/tty.SLAB_USBtoUART'

    lidar = YDLidar()
    lidar.open(port, 128000)
    lidar.restart()

    info = lidar.info()
    print(info)

    # lidar.start_motor()

    print(lidar.health())

    # lidar.start_motor()
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
