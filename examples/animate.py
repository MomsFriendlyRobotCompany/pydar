#!/usr/bin/env python2
'''Animates distances and measurment quality'''
from pydar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import logging

PORT_NAME = '/dev/tty.SLAB_USBtoUART'  # '/dev/ttyUSB0'
DMAX = 4000
IMIN = 0
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def update_line2(num, func, line):
    # scan = next(iterator)
    scan = func()
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def run():

    lidar = RPLidar(PORT_NAME)

    logging.getLogger().addHandler(logging.StreamHandler())

    logger = logging.getLogger('pydar.rplidar')
    logger.setLevel(logging.INFO)

    # lidar.open()
    lidar.start_motor()
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    # iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(
        fig,
        update_line2,
        # fargs=(iterator, line),
        fargs=(lidar.get, line),
        interval=50)

    plt.show()
    lidar.stop_motor()

if __name__ == '__main__':
    run()
