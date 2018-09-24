import serial
from struct import pack, unpack
import time
from collections import namedtuple


Info = namedtuple('Info', 'model firmware hw_ver sn')
Health = namedtuple('Health', 'status error')


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

        self.serial.flushInput()

        # self.serial.dtr = 1

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

    # def open()
    def stop(self):
        """
        When the system is in the scanning state, X4 has been sending out point
        cloud data. If you need to close the scan at this time, you can send
        this command to stop the system from scanning. After sending the stop
        command, the system will be in standby state. At this point, the
        device's ranging unit is in low power mode and the laser is off.

        The command is unresponsive, so the system will not reply with any
        message after receiving the command.
        """
        msg = [0xA5, 0x65]
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(0.1)

    def start_motor(self):
        self.serial.dtr = 1

    def stop_motor(self):
        self.serial.dtr = 0

    def scan(self):
        pass

    def get_all(self):
        print("valid scan message header")

        num_buf = self.serial.inWaiting()
        pkt = self.serial.read(num_buf)

        # print("raw pkt[{}]: {}".format(len(pkt), pkt))

        pkt = unpack('{}B'.format(num_buf), pkt)

        print("packet: {}".format([hex(x) for x in pkt]))

        # print(">> pkt data type", type(pkt))
        # if pkt[0] == 0xaa and pkt[1] == 0x55:
        ph = pkt[:2]
        if ph == (0xaa, 0x55,):
            pkt_type = pkt[2]
            num = pkt[3]
            start_angle = (pkt[5] << 8) + pkt[4]
            stop_angle = (pkt[7] << 8) + pkt[6]
            cksum = (pkt[9] << 8) + pkt[8]

            print('='*40)
            print('Start: {}      Stop: {}'.format(start_angle, stop_angle))
            print('checksum: {}'.format(cksum))
            print('Number: {}'.format(num))
            for i in range(num):
                s = self.serial.read(2)
                s = unpack('2B', s)
                print("  pt[{}]: {}".format(i, (s[1] << 8) + s[0]))
        else:
            print('*'*40)
            print("<<< Bad scan data >>>")
            dprint(pkt)
            print('*'*40)

    def get(self):
        ser_wait = self.serial.inWaiting()
        print(">> input buffer waiting:", ser_wait)
        if ser_wait > 0:
            self.serial.flushInput()

        msg = [0xA5, self.SCAN]  # 0xA560
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(0.1)
        pkt = self.serial.read(7)
        header = pkt
        d = unpack('7B', header)

        print("header[{}]: {}".format(len(d), [hex(x) for x in d]))  # packet: (165, 90, 5, 0, 0, 64, 129)

        if d == (self.HEADER, self.RESPONSE, 0x05, 0, 0, 0x40, 0x81):
            get_all()
        else:
            print('*'*40)
            print('<<< invalid pkt >>>')
            dprint(pkt)
            print('*'*40)


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

    # def valid_info_pkt(self, pkt):
    #     """
    #     [A5][5A][14][0][0][0][04][...content...]
    #     content = [0:model][1-2: FW][3:HW][4-19:SN]
    #     """
    #     # print('[', end=' ')
    #     # for d in pkt:
    #     #     print(unpack('B', d)[0], end=',')
    #     # print(']')
    #     dprint(pkt)
    #
    #     d = unpack('27B', pkt)
    #     if d[:7] == (self.HEADER, self.RESPONSE, 0x14, 0, 0, 0, 4):
    #         print('valid info packet')
    #         info = {
    #             'model': d[7],
    #             'firmware_version': d[8],
    #             'hw_version': d[9],
    #             'sn': ''.join(map(str, d[10:]))
    #         }
    #         return info
    #     else:
    #         print('crap: ', d)
    #     return None

    # def make_pkt(self, cmd, length, move, type_code):
    #     msg = [self.HEADER, cmd]

    def info(self):
        """
        When an external device sends a Get Device Information command to A4
        (A5 90), X4 will feedback the device's model, firmware version, and
        hardware version, and the device's factory serial number.

        [A5][5A][14][0][0][0][04][...content...]
        content = [0:model][1-2: FW][3:HW][4-19:SN]
        """
        msg = [0xA5, self.INFO]
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(0.1)
        info = self.serial.read(1024)

        if len(info) != 27:
            print("{}".format([hex(x) for x in info]))
            raise Exception("ydlidar.info: invalid length {}".format(len(info)))

        info = unpack('27B', info)

        if info[:7] == (0xA5,0x5A,0x14,0,0,0,0x04,):
            print('header valid')
        else:
            raise Exception("ydlidar.info: invalid header {}".format([hex(x) for x in info[:7]]))

        # breakout data
        msg = info[7:]
        model = msg[0]
        firmware = '.'.join(map(str,msg[1:3]))
        hw_ver = msg[3]
        sn = ''.join(map(str, msg[4:]))

        return Info(model, firmware, hw_ver, sn)

    def health(self):
        """
        When the external device sends the Get Device Health Status command
        (A5 91) to X4, X4 will feedback the device's status code.
        """
        msg = [0xa5, 0x91]
        msg = pack('2B', *msg)
        self.serial.write(msg)
        time.sleep(0.1)

        data = self.serial.read(10)
        data = unpack('10B', data)

        # print("raw[{}]: {}".format(len(data), [hex(x) for x in data]))

        if data[:7] == (0xa5,0x5a,0x03,0,0,0,0x06):
            msg = data[7:]
            # code = ['standbye', 'run', 'error'] ???
            status = msg[0]
            err = (msg[2] << 8) + msg[1]
        else:
            raise Exception('ydlidar.health: invalid header {}'.format([hex(x) for x in data]))

        return Health(status, err)

    def restart(self):
        """
        When the external device sends the Get Device command to A4 (A5 40),
        X4 will start a soft reboot and the system will restart. This command
        does not answer.
        """
        msg = [0xa5, 0x40]
        msg = pack('2B', *msg)
        self.serial.write(msg)
        time.sleep(0.1)
        data = self.serial.read(1024)
        print('reboot data[{}]========\n{}\n========================'.format(len(data), data.decode('utf8')))
