#!/usr/bin/env python

from __future__ import print_function, division
from pydar.ydlidar import YDLidar
from pydar import RPLidar
import time
# DTR - motor en

if __name__ == "__main__":
    lidar = YDLidar()
    # lidar = RPLidar()
    lidar.open("/dev/ttyUSB1", 128000)
    lidar.reboot()
    # lidar.serial.dtr = 0
    info = lidar.info()
    print(info)

    time.sleep(1)
    lidar.get()
    # lidar.serial.dtr = 1
    # info = lidar.info()
    # print(info)
    # lidar.serial.dtr = 0
    # info = lidar.info()
    # print(info)
