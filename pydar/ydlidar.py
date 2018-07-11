import serial


class Base(object):
    def __init__(self):
        self.serial = serial.Serial()

    def open(self, port, baud):
        self.serial.port = port
        self.serial.baudrate = baud
        self.serial.open()
        if self.serial.is_open is False:
            print("Couldn't open serial port {}".format(port))

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


class YDLidar(Base):
    HEADER = 0xA5
    RESPONCE = 0x5A
    START = 0x60
    STOP = 0x65
    INFO = 0x90
    HEALTH = 0x91
    REBOOT = 0x40

    def __init__(self):
        pass

    def get(self):
        pass

    def valid_scan_pkt(self, pkt):
        d = unpack('BBBBBBBBBB', pkt[:10])

    dev valid_info_pkt(self, pkt):
        """
        [A5][5A][14][0][0][0][04][...content...]
        content = [0:model][1-2: FW][3:HW][4-19:SN]
        """
        d = unpack('2BIBBHB16B', pkt)
        if d[:7] == [self.HEADER, self.RESPONSE, 14, 4]:
            print('valid info packet')
            info = {
                'model': d[4],
                'firmware_version': d[5],
                'hw_version': d[6],
                'sn': ''.join(map(str, d[7:]))
            }
            return info
        return None


    def make_pkt(self, cmd, length, move, type_code):

        msg = [HEADER, cmd]
        l

    def get_pkt(self, pkt):
        pass
