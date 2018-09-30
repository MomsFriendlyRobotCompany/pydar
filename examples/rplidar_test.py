#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import sys
from pydar import RPLidar
import platform
import time


if platform.system() == 'Linux':
    PORT_NAME = '/dev/ttyUSB0'
else:
    PORT_NAME = '/dev/tty.SLAB_USBtoUART'


def run():
    '''Main function'''
    lidar = RPLidar()
    lidar.open(PORT_NAME)
    lidar.start()

    try:
        while True:
            time.sleep(0.2)
            print(lidar.get())
    except KeyboardInterrupt:
        print('Stoping.')
        lidar.shutdown = True
    # lidar.stop()
    lidar.close()
    # outfile.close()

if __name__ == '__main__':
    run()
