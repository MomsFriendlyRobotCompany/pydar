# MIT License Kevin Walchko (c) 2018

from __future__ import division, print_function
from serial import Serial
import time
from struct import Struct


unpack_unsigned_byte = Struct('B').unpack
unpack_unsigned_40byte = Struct('40B').unpack
unpack_unsigned_41byte = Struct('41B').unpack
unpack_unsigned_2518byte = Struct('2518B').unpack


class LDS01(object):
    """
    42 bytes per 6 degree packet
    360 deg/6 deg = 60 packets
    42 bytes * 60 packets = 2520 bytes for a full 360
    """
    def __init__(self):
        self.serial = Serial()
        self.illum = [0] * 360
        self.dist = [0] * 360
        self.scan = [(0,0,)] * 360
        self.err_cnt = 0
        self.debug = False

    def __del__(self):
        if self.serial.is_open:
            self.close()

    def debug_print(self, s):
        if self.debug:
            print(s)

    def close(self):
        self.run(False)
        self.serial.close()

    def open(self, port):
        self.serial.port = port
        self.serial.baudrate = 230400  # default speed

        try:
            self.serial.open()
        except:
            raise Exception('LIDAR: could not open', port)

    def run(self, state):
        if state:
            self.serial.write(b"b")
        else:
            self.serial.write(b"e")
        time.sleep(0.25)

    def read6(self, bytes):
        illum = (bytes[1] << 8) + bytes[0]
        dist = (bytes[3] << 8) + bytes[2]
        return (illum, dist,)

    def parse_pkt(self, pkt):
        """
        index - where in the 360 array are we
        pkt - 42 byte packet

        distance:
            120 - 3500 mm
            0x0078 - 0x0dac mm

        packet of 6 degrees:
            0: sync 0xfa
            1: degree index, 0xa0-0xdb or 0-59
            2-3: rpm
            4-9: angle 0:
                intensity, distance, reserved
            10-15: angle 1
            16-21: angle 2
            22-27: angle 3
            28-33: angle 4
            34-39: angle 5
            40-41: checksum
        """
        # check packet
        if len(pkt) != 42:
            self.debug_print('error packet length')
            return 1
        if pkt[0] == 0xFA and (0xA0 <= pkt[1] and pkt[1] <= 0xDB):
            # chksum = (pkt[41] << 8) + pkt[40]
            cchksum = 0xFF - sum(pkt[:40]) & 0xff
            # debug_print('pkt:',hex(pkt[40]),' ',hex(pkt[41]), '  calc:', hex(cchksum))
            if cchksum != pkt[40] or cchksum != pkt[41]:
                self.debug_print('{} {} {}'.format('checksum error', pkt[40], pkt[41], cchksum))
                return 1
            angle = 6*(pkt[1] - 0xa0)
            # print('pkt angle:', angle)
            # rpm
            # rpm = (pkt[3] << 8) + pkt[2]
            for i in range(6):
                index = 4+i*6
                ii, dd = self.read6(pkt[index:index+4])  # the reserved bytes aren't used
                self.illum[angle + i] = ii
                self.dist[angle + i] = dd
                self.scan[angle + i] = (angle + i, dd,)
        else:
            self.debug_print('{} {} {}'.format('header error', hex(pkt[0]), hex(pkt[1])))
            return 1
        return 0

    # def find_packet(self, byte=0x00):
    #     # byte = 0x00
    #     while byte != 0xFA:  # dec: 250
    #         byte = self.serial.read(1)
    #         byte = unpack_unsigned_byte(byte)[0]
    #         # if byte != 0xfa:
    #         #     print('bad pkt:', byte)
    #
    #     byte = self.serial.read(1)
    #     byte = unpack_unsigned_byte(byte)[0]
    #     if 0xA0 > byte > 0xDB:
    #         self.err_cnt += 1
    #         print(self.err_cnt, 'invalid angle:', byte, hex(byte))
    #         return self.find_packet(byte)
    #
    #     data = self.serial.read(40)
    #     data = unpack_unsigned_40byte(data)
    #     data = (0xfa, byte) + data
    #     ret = self.parse_pkt(data)
    #     if ret:
    #         self.err_cnt += 1
    #         print(self.err_cnt, 'packet error')
    #     else:
    #         self.good_read += 1

    def read(self):
        """
        Each read returns 6 data points and it does this 60 times for 360 total
        data points
        """
        # reset data
        self.good_read = 0
        self.illum = [0] * 360
        self.dist = [0] * 360
        self.scan = [(0,0,)] * 360

        byte = 0x00
        while byte != 0xFA:  # dec: 250
            byte = self.serial.read(1)
            byte = unpack_unsigned_byte(byte)[0]

        byte = self.serial.read(1)
        byte = unpack_unsigned_byte(byte)[0]
        if 0xA0 > byte > 0xDB:
            self.err_cnt += 1
            self.debug_print('{} {} {} {}'.format(self.err_cnt, 'invalid angle:', byte, hex(byte)))
            # return self.find_packet(byte)
            return

        data = self.serial.read(2518)
        data = unpack_unsigned_2518byte(data)
        data = (0xfa, byte) + data

        for i in range(60):
            index = i*42
            err = self.parse_pkt(data[index:index+42])
            if err:
                self.err_cnt += 1
                self.debug_print('{} {}'.format('packet error angle index:', i))
                self.debug_print('{} {}'.format('pkt:', data[index-10:index+10]))
            else:
                self.good_read += 1

        return self.scan

        # print('good reads:', self.good_read)

    # def read2(self):
    #     # reset data
    #     self.good_read = 0
    #     self.illum = [0] * 360
    #     self.dist = [0] * 360
    #
    #     # read 360 degrees
    #     for i in range(60):
    #         self.find_packet()
    #     print('good reads:', self.good_read)
    #
    # def read(self):
    #     # reset data
    #     self.illum = [0] * 360
    #     self.dist = [0] * 360
    #
    #     # find start packet
    #     byte = 0x00
    #     while byte != 0xFA:  # dec: 250
    #         byte = self.serial.read(1)
    #         byte = unpack_unsigned_byte(byte)[0]
    #         # print('byte:', byte)
    #
    #     byte = self.serial.read(1)
    #     byte = unpack_unsigned_byte(byte)[0]
    #     if 0xA0 > byte > 0xDB:
    #         self.err_cnt += 1
    #         print(self.err_cnt, 'invalid angle:', byte, hex(byte))
    #         return None
    #     # else:
    #     #     print('angle:', byte, hex(byte))
    #
    #     data = self.serial.read(40)  # read remaining data
    #     data = unpack_unsigned_40byte(data)
    #     # print('40', data)
    #     data = (0xfa, byte,) + data
    #     ret = self.parse_pkt(data)
    #     if ret:
    #         self.err_cnt += 1
    #         print(self.err_cnt, 'packet error')
    #
    #     # read 360 degrees
    #     for i in range(1, 60):
    #         # print(i)
    #
    #         byte = 0x00
    #         while byte != 0xFA:  # dec: 250
    #             byte = self.serial.read(1)
    #             byte = unpack_unsigned_byte(byte)[0]
    #
    #         data = self.serial.read(41)
    #         data = unpack_unsigned_41byte(data)
    #         data = (0xfa,) + data
    #         ret = self.parse_pkt(data)
    #         if ret:
    #             self.err_cnt += 1
    #             print(self.err_cnt, 'packet error')
