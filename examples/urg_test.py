#!/usr/bin/env python3
# MIT License Kevin Walchko (c) 2018
#
# this needs: pip install pydar

from pydar import URG04LX
import time
from math import pi


if __name__ == '__main__':
	a = URG04LX()
	port = "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00"
	ok = a.init(port, baudrate=19200)
	if not ok:
		print("*** Couldn't init lidar ***")
		exit(1)

	print(a.pp_params)
	a.printInfo()

	theta = [i*2*pi/360 for i in range(360)]

	# plt.ion()
	for i in range(2):
		pts = a.capture()
		print('-'*40)
		print('distance points:', pts)
		# print('timestamp:', tm)
		print('number points:', len(pts.scan))

	a.close()
	time.sleep(3)
