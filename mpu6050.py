from smbus import SMBus


# ref: http://www.widesnow.com/entry/2015/09/10/061128
DEV_ADDR = 0x68
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b
PWR_MGMT_2 = 0x6c

class Mpu6050:

    def __init__(self):
        self.bus = SMBus(1)
        self.bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

    def __read_word(self, adr):
        high = self.bus.read_byte_data(DEV_ADDR, adr)
        low  = self.bus.read_byte_data(DEV_ADDR, adr+1)
        val  = (high << 8) + low
        return val

    def __read_word_sensor(self, adr):
        val  = self.__read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    #角速度(ジャイロ)データ取得
    def __load_gyro_sensor(self):
        x_gyro = self.__read_word_sensor(GYRO_XOUT)
        y_gyro = self.__read_word_sensor(GYRO_YOUT)
        z_gyro = self.__read_word_sensor(GYRO_ZOUT)
        return [x_gyro, y_gyro, z_gyro]

    def gyro_lsb(self):
        x_gyro,y_gyro,z_gyro = self.__load_gyro_sensor()
        x_gyro /= 131000
        y_gyro /= 131000
        z_gyro /= 131000
        return [x_gyro, y_gyro, z_gyro]

    #加速度データ取得
    def __load_accelerometer(self):
        x_accel = self.__read_word_sensor(ACCEL_XOUT)
        y_accel = self.__read_word_sensor(ACCEL_YOUT)
        z_accel = self.__read_word_sensor(ACCEL_ZOUT)
        return [x_accel, y_accel, z_accel]

    def accel_lsb(self):
        x_accel,y_accel,z_accel = self.__load_accelerometer()
        x_accel /= 16384000
        y_accel /= 16384000
        z_accel /= 16384000
        return [x_accel, y_accel, z_accel]