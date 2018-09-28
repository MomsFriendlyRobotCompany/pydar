import serial
from struct import pack, unpack
import time
from collections import namedtuple
from math import atan, pi


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
    # HEADER = 0xA5
    # RESPONSE = 0x5A
    # SCAN = 0x60
    # STOP = 0x65
    # INFO = 0x90
    # HEALTH = 0x91
    # REBOOT = 0x40

    def __init__(self):
        Base.__init__(self)
        # self.reboot()

    def __del__(self):
        # send packet to stop lidar
        # self.motor(False)
        # msg = [0xA5, self.STOP]
        # dmsg = pack('2B', *msg)
        # self.serial.write(dmsg)
        # time.sleep(0.25)
        self.stop()
        self.serial.close()
        print('YDLidar exiting')

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
        self.motor(False)

    def motor(self, val=True):
        """
        Turns on/off the motor which spins the lidar.
        """
        if val:
            self.serial.dtr = 1
        else:
            self.serial.dtr = 0

    def start(self):
        ser_wait = self.serial.inWaiting()
        print(">> input buffer waiting:", ser_wait)
        if ser_wait > 0:
            self.serial.flushInput()

        msg = [0xA5, 0x60]  # start scan: 0xA560
        dmsg = pack('2B', *msg)
        self.serial.write(dmsg)
        time.sleep(0.1)

    def angle(self, l, h):
        """
        Convert FSA and LSA to an angular degree
        """
        return (((h >> 1) << 8) + l) / 64

    def angle_correction(self, distance):
        if distance == 0:
            corr = 0
        else:
            corr = 180/pi*atan(21.8*(155.3-distance)/(155.3*distance))
        return corr

    def get_scan_pkt(self, pkt):
        ph = pkt[:2]
        # pkt_type =  pkt[2]
        if ph == (0xaa, 0x55,):
            pkt_type = pkt[2]
            num = pkt[3]
            start_angle = self.angle(*pkt[4:6])
            stop_angle = self.angle(*pkt[6:8])
            cksum = 0

            print('='*40)
            print('Start: {}      Stop: {}'.format(start_angle, stop_angle))
            print('checksum: {}'.format(cksum))
            print('Number: {}'.format(num))

            distance = []
            data = pkt[10:]

            if num*2 > len(data):
                print("*** not enough data[{}] == num[{}] ***".format(len(data), num))
                return None, None

            for i in range(num):
                dist = ((data[2*i + 1] << 8) + data[2*i])/4/1000  # meters
                distance.append(dist)

            fsac = self.angle_correction(distance[0])
            lsac = self.angle_correction(distance[-1])

            start_angle += fsac
            stop_angle += lsac
            da = (start_angle - stop_angle)/num
            angle = start_angle

            scan = []
            for r in distance:
                scan.append((angle, r,))
                angle += da
            return 10+2*num, scan

        else:
            print('*'*40)
            print("<<< Bad scan data >>>")
            dprint(pkt)
            print('*'*40)
            return None, None

    def get_all(self, pkt):
        """
        Grabs the streaming data ... should this be a thread with a buffer?

        pkt is unpacked data
        """
        # num_buf = self.serial.inWaiting()
        # pkt = self.serial.read(num_buf)

        # print("raw pkt[{}]: {}".format(len(pkt), pkt))
        # pkt = (170, 85, 0, 11, 1, 90, 19, 90, 184, 94, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 85, 0, 14, 21, 90, 45, 90, 146, 91, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 85, 0, 15, 47, 90, 73, 90, 204, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 85, 0, 4, 75, 90, 81, 90, 176, 81, 0, 0, 0, 0, 0, 0, 0, 0, 170, 85, 0, 40, 193, 101, 245, 106, 66, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 19, 200, 18, 140, 18, 76, 18, 12, 18, 108, 17, 52, 17, 88, 17, 40, 17, 236, 16, 184, 16, 132, 16, 88, 16, 36, 16, 248, 15, 204, 15, 160, 15, 116, 15, 76, 15, 36, 15, 248, 14, 208, 14, 172, 14, 136, 14, 96, 14, 64, 14, 28, 14, 252, 13, 170, 85, 0, 6, 23, 107, 189, 107, 20, 83, 220, 13, 192, 13, 164, 13, 136, 13, 112, 13, 84, 13, 170, 85, 0, 40, 5, 114, 147, 120, 36, 119, 60, 13, 32, 13, 32, 13, 0, 0, 248, 5, 244, 5, 252, 5, 4, 6, 20, 6, 0, 0, 0, 0, 0, 0, 0, 0, 220, 8, 240, 8, 16, 9, 52, 9, 84, 9, 120, 9, 160, 9, 196, 9, 236, 9, 208, 9, 0, 0, 0, 0, 176, 9, 164, 9, 160, 9, 144, 9, 124, 9, 0, 0, 0, 0, 0, 0, 8, 11, 252, 10, 244, 10, 236, 10, 228, 10, 232, 10, 0, 0, 170, 85, 0, 4, 191, 120, 65, 121, 36, 80, 92, 7, 88, 7, 72, 7, 60, 7, 170, 85, 0, 40, 217, 125, 59, 133, 154, 142, 56, 7, 64, 7, 0, 0, 0, 0, 164, 9, 160, 9, 152, 9, 148, 9, 144, 9, 140, 9, 136, 9, 136, 9, 128, 9, 124, 9, 124, 9, 92, 9, 40, 9, 0, 0, 0, 0, 0, 0, 190, 2, 0, 0, 68, 10, 68, 10, 56, 10, 88, 10, 84, 10, 84, 10, 80, 10, 84, 10, 84, 10, 88, 10, 84, 10, 64, 10, 16, 10, 0, 0, 0, 0, 144, 10, 136, 10, 128, 10, 170, 85, 0, 5, 109, 133, 49, 134, 154, 89, 112, 10, 112, 10, 108, 10, 116, 10, 116, 10, 170, 85, 0, 40, 1, 138, 233, 145, 98, 102, 120, 10, 124, 10, 132, 10, 140, 10, 164, 10, 0, 0, 0, 0, 156, 10, 164, 10, 176, 10, 180, 10, 188, 10, 192, 10, 200, 10, 208, 10, 216, 10, 228, 10, 236, 10, 240, 10, 252, 10, 244, 10, 200, 10, 0, 0, 80, 11, 52, 11, 60, 11, 72, 11, 84, 11, 96, 11, 108, 11, 124, 11, 136, 11, 148, 11, 164, 11, 180, 11, 192, 11, 212, 11, 228, 11, 248, 11, 0, 0, 170, 85, 0, 5, 33, 146, 243, 146, 192, 86, 0, 0, 164, 6, 168, 6, 180, 6, 0, 0, 170, 85, 0, 40, 45, 150, 145, 158, 190, 126, 0, 0, 0, 0, 0, 0, 184, 12, 204, 12, 228, 12, 252, 12, 4, 13, 68, 13, 64, 13, 84, 13, 0, 0, 0, 0, 40, 9, 60, 9, 0, 0, 0, 0, 0, 0, 0, 0, 52, 13, 36, 13, 248, 12, 228, 12, 192, 12, 164, 12, 140, 12, 112, 12, 80, 12, 60, 12, 32, 12, 8, 12, 240, 11, 216, 11, 192, 11, 192, 11, 148, 11, 148, 11, 108, 11, 88, 11, 72, 11, 170, 85, 0, 2, 201, 158, 3, 159, 112, 86, 52, 11, 36, 11, 170, 85, 0, 40, 191, 161, 139, 170, 146, 117, 16, 11, 0, 11, 236, 10, 220, 10, 204, 10, 192, 10, 176, 10, 176, 10, 144, 10, 132, 10, 120, 10, 104, 10, 92, 10, 80, 10, 68, 10, 60, 10, 44, 10, 32, 10, 24, 10, 16, 10, 4, 10, 252, 9, 240, 9, 232, 9, 224, 9, 216, 9, 208, 9, 200, 9, 192, 9, 184, 9, 180, 9, 172, 9, 164, 9, 160, 9, 152, 9, 144, 9, 140, 9, 136, 9, 128, 9, 124, 9, 170, 85, 0, 4, 199, 170, 117, 171, 12, 80, 124, 9, 116, 9, 112, 9, 108, 9, 170, 85, 0, 1, 215, 173, 215, 173, 194, 93, 104, 9, 170, 85, 1, 1, 27, 174, 27, 174, 195, 93, 104, 9, 170, 85, 0, 40, 89, 174, 109, 3, 74, 208, 100, 9, 96, 9, 96, 9, 92, 9, 92, 9, 88, 9, 88, 9, 84, 9, 84, 9, 84, 9, 84, 9, 80, 9, 80, 9, 80, 9, 84, 9, 84, 9, 84, 9, 84, 9, 88, 9, 88, 9, 88, 9, 92, 9, 92, 9, 96, 9, 96, 9, 100, 9, 100, 9, 104, 9, 108, 9, 112, 9, 116, 9, 120, 9, 124, 9, 128, 9, 136, 9, 140, 9, 144, 9, 148, 9, 156, 9, 160, 9, 170, 85, 0, 12, 165, 3, 57, 6, 50, 92, 168, 9, 172, 9, 184, 9, 188, 9, 196, 9, 204, 9, 208, 9, 220, 9, 228, 9, 236, 9, 244, 9, 252, 9, 170, 85, 0, 34, 199, 8, 173, 16, 8, 104, 4, 10, 16, 10, 28, 10, 36, 10, 48, 10, 60, 10, 68, 10, 80, 10, 92, 10, 104, 10, 116, 10, 132, 10, 144, 10, 156, 10, 168, 10, 184, 10, 200, 10, 212, 10, 228, 10, 244, 10, 4, 11, 20, 11, 40, 11, 56, 11, 72, 11, 92, 11, 112, 11, 128, 11, 152, 11, 168, 11, 196, 11, 212, 11, 232, 11, 4, 12, 170, 85, 0, 40, 35, 18, 145, 27, 72, 121, 28)

        # pkt = unpack('{}B'.format(num_buf), pkt)

        # print(">> scan header: {}".format([hex(x) for x in pkt[:10]]))

        index = 0
        scans = []

        while index < len(pkt):
            if 12 > len(pkt[index:]):
                print("*** not enough data[{}] for scan message ***".format(len(pkt[index:])))
                return scans

            offset, scan = self.get_scan_pkt(pkt[index:])

            # need more data from serial port
            if offset is None:
                return scans
            else:
                scans += scan
                index += offset
                # print(">> scan header: {}".format([hex(x) for x in pkt[:10]]))
                # print(">> data[:{}]: {}".format(index, scan))

    def get(self):
        # ser_wait = self.serial.inWaiting()
        # print(">> input buffer waiting:", ser_wait)
        # if ser_wait > 0:
        #     self.serial.flushInput()
        #
        # msg = [0xA5, 0x60]  # start scan: 0xA560
        # dmsg = pack('2B', *msg)
        # self.serial.write(dmsg)
        # time.sleep(0.1)

        # read what is in the buffer
        pkt = self.serial.read(1024)
        d = unpack('{}B'.format(len(pkt)), pkt)

        # print(d)
        scan = None

        print("\npacket[{}], header: {}\n".format(len(d), [hex(x) for x in d[:7]]))  # packet: (165, 90, 5, 0, 0, 64, 129)

        if d[:7] == (0xA5, 0x5A, 0x05, 0, 0, 0x40, 0x81,):
            """
            Per the documentation, you ignore the response length. As long as
            this header is good, then grab the data which will stream until
            stop() is called.
            """
            print("valid scan message header")
            scan = self.get_all(d[7:])
        else:
            print('*'*40)
            print('<<< invalid pkt >>>')
            print(d)
            print('*'*40)

        return scan

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
        data = self.serial.read(1024)  # first byte is 0xA5/165 and second is 0x40/64
        print('reboot data[{}]========\n{}\n========================'.format(len(data), data[2:].decode('utf8')))
