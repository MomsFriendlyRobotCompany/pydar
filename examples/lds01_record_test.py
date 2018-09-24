#!/usr/bin/env python3
# MIT License Kevin Walchko (c) 2018
#
# this needs: pip install the-collector
# this needs: pip install pydar

from pydar import LDS01
import time
from math import pi
from the_collector.bagit import BagWriter


if __name__ == '__main__':
	a = LDS01()
	a.open("/dev/tty.SLAB_USBtoUART")
	a.run(True)

	# this file name gives a time/date when it was created
	# you don't have to do this, 'data.json' would work fine too
	# filename = 'robot-{}.json'.format(time.ctime().replace(' ', '-'))
	filename = 'test.json'

	# create the writer
	bag = BagWriter()
	bag.open(['lidar'])

	for i in range(100):
		pts = a.read()
		bag.push('lidar', pts)
		if (i % 20) == 0: print(i)
		# print('-'*40)
		# print('distance points:', pts)
		# print('number points:', len(pts))

	bag.write(filename)  # actually writes the data to disk

	a.close()
	time.sleep(3)
