
# import logging
import sys
import time
import codecs
import serial
import struct
from struct import pack, unpack
from threading import Thread

SYNC_BYTE = b'\xA5'
SYNC_BYTE2 = b'\x5A'

GET_INFO_BYTE = b'\x50'
GET_HEALTH_BYTE = b'\x52'

STOP_BYTE = b'\x25'
RESET_BYTE = b'\x40'

SCAN_BYTE = b'\x20'
FORCE_SCAN_BYTE = b'\x21'

DESCRIPTOR_LEN = 7
INFO_LEN = 20
HEALTH_LEN = 3

INFO_TYPE = 4
HEALTH_TYPE = 6
SCAN_TYPE = 129

#Constants & Command to start A2 motor
MAX_MOTOR_PWM = 1023
DEFAULT_MOTOR_PWM = 660
SET_PWM_BYTE = b'\xF0'

_HEALTH_STATUSES = {
    0: 'Good',
    1: 'Warning',
    2: 'Error',
}

PY3 = True if (int(sys.version[0]) == 3) else False


class RPLidarException(Exception):
    '''Basic exception class for RPLidar'''


def _b2i(byte):
    '''Converts byte to integer (for Python 2 compatability)'''
    return byte if PY3 else ord(byte)

# def _process_scan(raw):
#     '''Processes input raw data and returns measurment data'''
#     new_scan = bool(_b2i(raw[0]) & 0b1)
#     inversed_new_scan = bool((_b2i(raw[0]) >> 1) & 0b1)
#     quality = _b2i(raw[0]) >> 2
#     if new_scan == inversed_new_scan:
#         raise RPLidarException('New scan flags mismatch')
#     check_bit = _b2i(raw[1]) & 0b1
#     if check_bit != 1:
#         raise RPLidarException('Check bit not equal to 1')
#     angle = ((_b2i(raw[1]) >> 1) + (_b2i(raw[2]) << 7)) / 64.
#     distance = (_b2i(raw[3]) + (_b2i(raw[4]) << 8)) / 4.
#     return (new_scan, quality, angle, distance,)


class RPLidar(object):
    '''Class for communicating with RPLidar rangefinder scanners'''

    # serial = None  #: serial port connection
    # port = ''  #: Serial port name, e.g. /dev/ttyUSB0
    timeout = 1  #: Serial port timeout
    # motor = False  #: Is motor running?
    baudrate = 115200  #: Baudrate for serial port

    # def __init__(self, port, baudrate=115200, timeout=1, logger=None):
    def __init__(self):
        '''Initilize RPLidar object for communicating with the sensor.

        Parameters
        ----------
        port : str
            Serial port name to which sensor is connected
        baudrate : int, optional
            Baudrate for serial connection (the default is 115200)
        timeout : float, optional
            Serial port connection timeout in seconds (the default is 1)
        logger : logging.Logger instance, optional
            Logger instance, if none is provided new instance is created
        '''
        self.serial = None
        # self.port = port
        # self.baudrate = baudrate
        # self.timeout = timeout
        self.motor_running = False
        # if logger is None:
        #     logger = logging.getLogger(__name__)
        # self.logger = logger
        # self.open(port, baudrate, timeout)
        # self.start_motor()

        self.scan = [(0,0,)]*360
        self.shutdown = False
        self.new_scan = False

    def __del__(self):
        # self.stop()
        self.shutdown = True
        if self.serial:
            self.motor(False)
            time.sleep(1)
            self.close()
        print("bye")

    def start(self):
        self.shutdown = False
        t = Thread(target=self.update2, name="rplidar", args=())
        t.daemon = True
        t.start()
        return

    def open(self, port):
        '''Connects to the serial port with the name `self.port`. If it was
        connected to another serial port disconnects from it first.'''
        if self.serial:
            self.close()
        try:
            self.serial = serial.Serial(
                port,
                self.baudrate,
                # parity=serial.PARITY_NONE,
                # stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                dsrdtr=False)

            self.motor(False)

        except serial.SerialException as err:
            raise RPLidarException('Failed: %s' % err)
        # self.reset()

    def close(self):
        '''Disconnects from the serial port'''
        self.stop()
        self.shutdown = True
        # self.motor(False)
        # time.sleep(2)
        if self.serial.is_open:
            self.serial.close()
            self.serial = None

    def set_pwm(self, pwm):
        # assert(0 <= pwm <= MAX_MOTOR_PWM)
        if 0 > pwm > MAX_MOTOR_PWM:
            print('invalid pwm')
        payload = struct.pack("<H", pwm)  # little endian, unsigned short
        # self._send_payload_cmd(SET_PWM_BYTE, payload)

        cmd = SET_PWM_BYTE

        size = struct.pack('B', len(payload))
        req = SYNC_BYTE + cmd + size + payload
        checksum = 0
        for v in struct.unpack('B'*len(req), req):
            checksum ^= v
        req += struct.pack('B', checksum)
        self.serial.write(req)

    # def start_motor(self):
    #     '''Starts sensor motor'''
    #     if self.motor_running:
    #         return
    #     # self.logger.info('Starting motor')
    #     # For A1
    #     self.serial.dtr = False
    #     # For A2
    #     self.set_pwm(DEFAULT_MOTOR_PWM)
    #     self.motor_running = True
    #
    # def stop_motor(self):
    #     '''Stops sensor motor'''
    #     if not self.motor_running:
    #         return
    #     # self.logger.info('Stoping motor')
    #     # For A2
    #     self.set_pwm(0)
    #     time.sleep(.001)
    #     # For A1
    #     self.serial.dtr = True
    #     self.motor_running = False

    def motor(self, value):
        if value:
            # if self.motor_running:
            #     return
            # For A1
            self.serial.dtr = False
            # For A2
            # self.set_pwm(DEFAULT_MOTOR_PWM)
            # self.set_pwm(0)
            # self.motor_running = True
        else:
            # if not self.motor_running:
            #     return
            # self.logger.info('Stoping motor')
            # For A2
            # self.set_pwm(0)
            # time.sleep(.001)
            # For A1
            self.serial.dtr = True
            # self.motor_running = False

    # def _send_payload_cmd(self, cmd, payload):
    #     '''Sends `cmd` command with `payload` to the sensor'''
    #     size = struct.pack('B', len(payload))
    #     req = SYNC_BYTE + cmd + size + payload
    #     checksum = 0
    #     for v in struct.unpack('B'*len(req), req):
    #         checksum ^= v
    #     req += struct.pack('B', checksum)
    #     self.serial.write(req)

    def info(self):
        '''Get device information

        Returns
        -------
        dict
            Dictionary with the sensor information
        '''
        # msg = [0xA5, 0x50]
        # dmsg = pack('2B', *msg)
        # self.serial.write(dmsg)
        self.write([0xA5, 0x50])
        # self._send_cmd(GET_INFO_BYTE)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != INFO_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != INFO_TYPE:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        d = unpack('20B', raw)
        serialnumber = codecs.encode(raw[4:], 'hex').upper()
        serialnumber = codecs.decode(serialnumber, 'ascii')
        # data = {
        #     'model': _b2i(raw[0]),
        #     'firmware': (_b2i(raw[2]), _b2i(raw[1])),
        #     'hardware': _b2i(raw[3]),
        #     'serialnumber': serialnumber,
        # }
        data = {
            'model': d[0],
            'firmware': (d[2], d[1]),
            'hardware': d[3],
            'serialnumber': serialnumber,
        }
        return data

    def health(self):
        '''Get device health state. When the core system detects some
        potential risk that may cause hardware failure in the future,
        the returned status value will be 'Warning'. But sensor can still work
        as normal. When sensor is in the Protection Stop state, the returned
        status value will be 'Error'. In case of warning or error statuses
        non-zero error code will be returned.

        Returns
        -------
        status : str
            'Good', 'Warning' or 'Error' statuses
        error_code : int
            The related error code that caused a warning/error.
        '''
        self._send_cmd(GET_HEALTH_BYTE)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != HEALTH_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != HEALTH_TYPE:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        status = _HEALTH_STATUSES[_b2i(raw[0])]
        error_code = (_b2i(raw[1]) << 8) + _b2i(raw[2])
        return status, error_code

    def stop(self):
        '''Stops scanning process, disables laser diode and the measurment
        system, moves sensor to the idle state.'''
        # self.logger.info('Stoping scanning')
        # self._send_cmd(STOP_BYTE)
        # time.sleep(.001)

        # msg = [0xa5, 0x25]
        # msg = pack('2B', *msg)
        # self.serial.write(msg)
        self.write([0xa5, 0x25])
        time.sleep(0.1)

        self.motor(False)
        # self.clear_input()
        # self.serial.read_all()

    def write(self, msg):
        msg = pack('B'*len(msg), *msg)
        self.serial.write(msg)

    def read(self, many=1024):
        raw = self.serial.read(many)
        print('read', len(raw))
        data = unpack('B'*len(raw), raw)
        return data

    def reset(self):
        """
        Resets sensor core, reverting it to a similar state as it has
        just been powered up. Should return something like this:

        RP LIDAR System.
        Firmware Ver 1.24 - rc0, HW Ver 5
        Model: 18
        """
        # msg = [0xa5, 0x40]
        # msg = pack('2B', *msg)
        # self.serial.write(msg)
        self.write([0xa5, 0x40])
        time.sleep(0.5)
        # data = self.serial.read(1024)  # first byte is 0xA5/165 and second is 0x40/64
        # offset = 0
        # # print(data)
        #
        # b = unpack('{}B'.format(len(data)), data)
        # # print(b)
        #
        # for i, p in enumerate(b):
        #     if p == 82 and b[i+1] == 80:
        #         offset = i
        # data = data[offset:]
        # # print('offset', offset)
        # # print('>> ',len(data))
        # print('reboot data[{}]========\n{}\n========================'.format(len(data), data.decode('utf8')))
        # time.sleep(1)

    def get(self, max_buf_meas=500):
        '''Iterate over measurments. Note that consumer must be fast enough,
        otherwise data will be accumulated inside buffer and consumer will get
        data with increaing lag.

        Parameters
        ----------
        max_buf_meas : int
            Maximum number of measurments to be stored inside the buffer. Once
            numbe exceeds this limit buffer will be emptied out.

        Yields
        ------
        new_scan : bool
            True if measurment belongs to a new scan
        quality : int
            Reflected laser pulse strength
        angle : float
            The measurment heading angle in degree unit [0, 360)
        distance : float
            Measured object distance related to the sensor's rotation center.
            In millimeter unit. Set to 0 when measurment is invalid.
        '''
        while not self.new_scan:
            time.sleep(0.001)
        self.new_scan = False
        return self.scan

    def getscan(self, data):
        """
        Once the header packet is found, this functions pulls the measurements
        until it runs out of usable bytes.

        return: left over bytes
        """
        offset = 0
        len_data = len(data)
        while (offset + 5) <= len_data:
            pkt = data[offset:offset+5]
            start = ((1 & pkt[0]) == 1)
            if start:
                print("====[start]==================================")
                self.scan = [(0,0,)]*360
                self.new_scan = True
            chkS = (((pkt[0] & 2) >> 1) != (1 & pkt[0]))
            if not chkS:
                # print(((pkt[0] & 2) >> 1))
                # print(1 & pkt[0])
                # raise Exception('S')
                print('S error')
                offset += 5
                continue
            if not (pkt[1] & 1):
                # raise Exception('C')
                print('C error')
                offset += 5
                continue
            q = (pkt[0] >> 2)
            angle = ((pkt[2] << 7) + (pkt[1] >> 1))/64
            dist = ((pkt[4] << 8) + pkt[3])/4  # mm
            offset += 5
            # print('range',angle, dist, q)
            try:
                index = int(round(angle))
                if index >= 360:
                    index = 0
                self.scan[index] = (angle, dist)
            except Exception as e:
                print(e)
                print('angle:', angle, int(round(angle)))
                exit(1)

        return data[offset:]

    def update2(self):
        # max_buf_meas=500  # ??
        self.motor(True)
        self.reset()
        self.serial.reset_input_buffer()
        # msg = [0xa5, 0x20]
        # msg = pack('2B', *msg)
        # self.serial.write(msg)
        self.write([0xa5, 0x20])
        time.sleep(0.001)

        self.scan = [(0,0,)]*360
        # done = False

        # raw = self.serial.read(1024)
        # print('read', len(raw))
        # data = unpack('{}B'.format(len(raw)), raw)
        data = self.read()
        offset = 0
        data_len = len(data)

        # find header
        while True:
            if data[offset:offset+7] == (0xa5,0x5a,0x05,0,0,0x40,0x81,):
                print("==[header found]==")
                offset += 7
                break
            # print([hex(x) for x in data[offset:offset+7]])
            offset += 1
            if (offset+7) > data_len:
                print('offset', offset, 'send scan again')
                # raise Exception("out of data")
                # msg = [0xa5, 0x20]
                # msg = pack('2B', *msg)
                # self.serial.write(msg)
                self.write([0xa5, 0x20])
                time.sleep(0.001)

                # raw = self.serial.read(1024)
                # print('read', len(raw))
                # data = unpack('{}B'.format(len(raw)), raw)
                data = self.read()
                offset = 0
                data_len = len(data)

        # get scan packets
        while not self.shutdown:
            rest = self.getscan(data[offset:])

            # raw = self.serial.read(1024)
            # print('read', len(raw))
            # data = rest + unpack('{}B'.format(len(raw)), raw)
            data = rest + self.read()
            offset = 0

            print('buffer clear')
