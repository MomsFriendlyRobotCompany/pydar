
from __future__ import print_function, division
import serial
from struct import pack, unpack
import time


class Base(object):
    def __init__(self):
        self.serial = serial.Serial()

    def open(self, port, baud, timeout=0.1):
        self.serial.port = port
        self.serial.baudrate = baud
        self.serial.timeout = timeout
        self.serial.open()
        self.serial.dtr = 0
        if self.serial.is_open is False:
            print("Couldn't open serial port {}".format(port))
        else:
            print(self.serial)

        self.serial.dtr = 1

        # seems to be the only way to correct the system between runs
        # msg = [0xA5, self.REBOOT]
        # dmsg = pack('2B', *msg)
        # self.serial.write(dmsg)
        # time.sleep(0.25)
        #
        # data = self.serial.read(1024)
        # print(data)

    def close(self):
        self.serial.close()

    def get(self):
        raise NotImplementedError()

    def health(self):
        raise NotImplementedError()

    def info(self):
        raise NotImplementedError()

    def start(self):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

def dprint(pkt):
    b = unpack('{}B'.format(len(pkt)), pkt)
    h = map(hex, b)
    print('pkt[{}]: {}'.format(len(pkt), h))


class YDLidar(Base):
    HEADER = 0xA5
    RESPONSE = 0x5A
    SCAN = 0x60
    STOP = 0x65
    INFO = 0x90
    HEALTH = 0x91
    REBOOT = 0x40

    def __init__(self):
        Base.__init__(self)
        # self.reboot()

    def __del__(self):
        # send packet to stop lidar
        self.stop_motor()
        msg = [0xA5, self.STOP]
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(0.25)
        self.serial.close()

    def start_motor(self):
        self.serial.dtr = 1

    def stop_motor(self):
        self.serial.dtr = 0

    def get(self):
        ser_wait = self.serial.inWaiting()
        print(">> input buffer waiting:", ser_wait)
        if ser_wait > 0:
            self.serial.flushInput()

        msg = [0xA5, self.SCAN]
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(0.1)
        pkt = self.serial.read(7)
        header = pkt
        d = unpack('7B', header)
        if d == (self.HEADER, self.RESPONSE, 0x05, 0, 0, 0x40, 0x81):
            print("valid scan message header")

            pkt = self.serial.read(10)
            pkt = unpack('10B', header)
            # print(">> pkt data type", type(pkt))
            # if pkt[0] == 0xaa and pkt[1] == 0x55:
            ph = pkt[:2]
            if ph == (0xaa, 0x55,):
                pkt_type = pkt[2]
                num = pkt[3]
                start_angle = pkt[4:6]
                stop_angle = pkt[6:8]
                cksum = pkt[8:]

                for i in range(num):
                    s = self.serial.read(2)
                    print("pt[{}]: {}".format(i, (s[1] << 8) + s[0]))
            else:
                print("Bad scan data")
                dprint(pkt)
        else:
            dprint(pkt)


    def valid_scan_pkt(self, pkt):
        dprint(pkt)
        header = pkt[:7]
        d = unpack('7B', header)
        print("valid scan message header:", d == (self.HEADER, self.RESPONSE, 0x05, 0, 0, 0x40, 0x81))

        # tell it to send
        print('-'*40)
        # msg = [0xA5, self.SCAN]
        # dmsg = pack('2B', *msg)
        # self.serial.write(dmsg)
        # pkt = []
        # while len(pkt) == 0:
        #     pkt = self.serial.read(1024)

        # dprint(pkt)
        # actual scan header
        header = pkt[7:16]
        print('header', len(header))
        d = unpack('10B', header)
        print('scan header:', d)

    def valid_info_pkt(self, pkt):
        """
        [A5][5A][14][0][0][0][04][...content...]
        content = [0:model][1-2: FW][3:HW][4-19:SN]
        """
        # print('[', end=' ')
        # for d in pkt:
        #     print(unpack('B', d)[0], end=',')
        # print(']')
        dprint(pkt)

        d = unpack('27B', pkt)
        if d[:7] == (self.HEADER, self.RESPONSE, 0x14, 0, 0, 0, 4):
            print('valid info packet')
            info = {
                'model': d[7],
                'firmware_version': d[8],
                'hw_version': d[9],
                'sn': ''.join(map(str, d[10:]))
            }
            return info
        else:
            print('crap: ', d)
        return None

    def make_pkt(self, cmd, length, move, type_code):
        msg = [self.HEADER, cmd]

    def info(self):
        msg = [0xA5, self.INFO]
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(1)
        info = self.serial.read(1024)
        # print('>> data:', info)
        # print('>> len:', len(info))
        data = None
        if info:
            data = self.valid_info_pkt(info)
        return data

    def get_pkt(self, pkt):
        pass

    def reboot(self):
        # seems to be the only way to correct the system between runs
        msg = [0xA5, self.REBOOT]
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(0.25)

        data = self.serial.read(1024)
        print('reboot data[{}]: {}'.format(len(data), data))
