


class Base(object):
    debug = False
    baudrate = 0
    
    def __del__(self):
        if self.serial.is_open:
            self.close()

    def debug_print(self, s):
        if self.debug:
            print(s)

    def read(self):
        pass

    def run(self):
        pass

    def open(self, port, baudrate=None):
        if baudrate is None:
            baudrate = self.baudrate

        self.serial.port = port
        self.serial.baudrate = baudrate  # default speed

        try:
            self.serial.open()
        except:
            raise Exception('LIDAR: could not open', port)

    def close(self):
