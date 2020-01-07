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
        low = self.bus.read_byte_data(DEV_ADDR, adr+1)
        val = (high << 8) + low
        return val

    def __read_word_sensor(self, adr):
        val = self.__read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    # 角速度(rad/msec)取得
    def __load_gyro_sensor(self):
        gyro_x = self.__read_word_sensor(GYRO_XOUT)
        gyro_y = self.__read_word_sensor(GYRO_YOUT)
        gyro_z = self.__read_word_sensor(GYRO_ZOUT)
        return [gyro_x, gyro_y, gyro_z]

    def gyro_lsb(self):
        gyro_x, gyro_y, gyro_z = self.__load_gyro_sensor()
        gyro_x /= 131
        gyro_y /= 131
        gyro_z /= 131
        return [gyro_x, gyro_y, gyro_z]

    # 加速度(m/msec)取得
    def __load_accelerometer(self):
        accel_x = self.__read_word_sensor(ACCEL_XOUT)
        accel_y = self.__read_word_sensor(ACCEL_YOUT)
        accel_z = self.__read_word_sensor(ACCEL_ZOUT)
        return [accel_x, accel_y, accel_z]

    def accel_lsb(self):
        x_accel, y_accel, z_accel = self.__load_accelerometer()
        x_accel /= 16384
        y_accel /= 16384
        z_accel /= 16384
        return [x_accel, y_accel, z_accel]
