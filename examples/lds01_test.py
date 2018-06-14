#!/usr/bin/env python2
# MIT License Kevin Walchko (c) 2018
#
# this needs: pip install pydar
# this needs: pip install matplotlib

from __future__ import division
from __future__ import print_function
from pydar import LDS01
import matplotlib.pyplot as plt
import time
from math import pi


if __name__ == '__main__':
	a = LDS01()
	# a.debug = True
	a.open("/dev/tty.SLAB_USBtoUART")
	a.run(True)

	theta = [i*2*pi/360 for i in range(360)]

	plt.ion()
	try:
		print("Press ctrl-C to stop")
		time.sleep(1)
		while True:
			pts = a.read()
			plt.subplot(111, projection='polar')
			plt.plot(theta, pts)
			plt.grid(True)
			plt.draw()
			plt.pause(0.2)
			plt.clf()

			# print('-'*40)
			# print('distance points:', pts)
			# print('number points:', len(pts))
	except KeyboardInterrupt:
		print("bye ...")

	a.close()
