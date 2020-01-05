from smbus import SMBus

class Mpu6050:

    def __init__(self):
        self.DEV_ADDR = 0x68 #https://shizenkarasuzon.hatenablog.com/entry/2019/03/06/114248
        self.ACCEL_XOUT = 0x3b
        self.ACCEL_YOUT = 0x3d
        self.ACCEL_ZOUT = 0x3f
        self.TEMP_OUT = 0x41
        self.GYRO_XOUT = 0x43
        self.GYRO_YOUT = 0x45
        self.GYRO_ZOUT = 0x47
        self.PWR_MGMT_1 = 0x6b
        self.PWR_MGMT_2 = 0x6c
        self.bus = SMBus(1)
        self.bus.write_byte_data(self.DEV_ADDR, self.PWR_MGMT_1, 0)

    def read_byte(self, adr):
        return self.bus.read_byte_data(self.DEV_ADDR, adr)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.DEV_ADDR, adr)
        low  = self.bus.read_byte_data(self.DEV_ADDR, adr+1)
        val  = (high << 8) + low
        return val

    def read_word_sensor(self, adr):
        val  = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    #角速度(ジャイロ)データ取得
    def get_gyro_data_lsb(self):
        self.x = self.read_word_sensor(self.GYRO_XOUT)
        self.y = self.read_word_sensor(self.GYRO_YOUT)
        self.z = self.read_word_sensor(self.GYRO_ZOUT)
        return [self.x, self.y, self.z]

    def get_gyro_data_deg(self):
        x,y,z = self.get_gyro_data_lsb()
        x = self.x / 1310
        y = self.y / 1310
        z = self.z / 1310
        return [x, y, z]

    #加速度データ取得
    def get_accel_data_lsb(self):
        self.x = self.read_word_sensor(self.ACCEL_XOUT)
        self.y = self.read_word_sensor(self.ACCEL_YOUT)
        self.z = self.read_word_sensor(self.ACCEL_ZOUT)
        return [self.x, self.y, self.z]

    def get_accel_data_g(self):
        x,y,z = self.get_accel_data_lsb()
        x = self.x / 16384.0
        y = self.y / 16384.0
        z = self.z / 16384.0
        return [x, y, z]