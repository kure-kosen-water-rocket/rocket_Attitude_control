import smbus
import math
import time
import csv
from servo import ServoController


DEV_ADDR = 0x68 #https://shizenkarasuzon.hatenablog.com/entry/2019/03/06/114248
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b
PWR_MGMT_2 = 0x6c

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def read_byte(adr):
    return bus.read_byte_data(DEV_ADDR, adr)

def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low  = bus.read_byte_data(DEV_ADDR, adr+1)
    val  = (high << 8) + low
    return val

def read_word_sensor(adr):
    val  = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

#角速度(ジャイロ)データ取得
def get_gyro_data_lsb():
    x = read_word_sensor(GYRO_XOUT)
    y = read_word_sensor(GYRO_YOUT)
    z = read_word_sensor(GYRO_ZOUT)
    return [x, y, z]

def get_gyro_data_deg():
    x,y,z = get_gyro_data_lsb()
    x = x / 1310
    y = y / 1310
    z = z / 1310
    return [x, y, z]

#加速度データ取得
def get_accel_data_lsb():
    x = read_word_sensor(ACCEL_XOUT)
    y = read_word_sensor(ACCEL_YOUT)
    z = read_word_sensor(ACCEL_ZOUT)
    return [x, y, z]

def get_accel_data_g():
    x,y,z = get_accel_data_lsb()
    x = x / 16384.0
    y = y / 16384.0
    z = z / 16384.0
    return [x, y, z]

#初期値の設定
dt = 0
total_time = 0
old_y_gyro = 0
old_angle = 0
fieldnames = ['x_accel', 'y_accel', 'z_accel', 'x_gyro', 'y_gyro', 'z_gyro', 'dt']
data_log = []
CALC_TIME = 5

Servo = ServoController(Pin=18)
Servo.set_position(90) #初期位置を90度に設定

while 1:
    start = time.time() #ループ中にかかる時間を計測開始

    x_gyro,  y_gyro,  z_gyro  = get_gyro_data_deg()
    x_accel, y_accel, z_accel = get_accel_data_g()

    data_log.append([x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro, dt])

    y_angle = math.atan2(x_accel , math.sqrt(y_accel**2 + z_accel**2))#姿勢角の算出
    k = 0.2 * (65536**-((abs(x_accel))/10)) #加速度が増加→加速度から得られる角度の比重を小さくしていく係数k
    y_angle = (((1-k) * (old_angle + (((old_y_gyro + y_gyro)* dt) / 2)) + ((k* y_angle)))) #加速度とジャイロの相補フィルター
    old_angle = y_angle #角度の変化量を足していくため一つ前の角度が必要
    old_y_gyro = y_gyro #台形積分用に一つ前の角速度を残しておく

    if int(total_time) % 1 == 0: #dt秒毎に処理を実行
        if 86-int(math.degrees(y_angle)) < 180 and 86-int(math.degrees(y_angle)) > 0: #0度~180度の時のみ動作
            Servo.set_position(86 - int(math.degrees(y_angle))) #初期位置は90度からなので(86は調整した値)

    if int(total_time) > int(CALC_TIME): #計測時間がtotal_timeに達したらプログラム終了
        break

    dt = time.time() - start #ループにかかる時間を微小時間dtに代入
    total_time += dt

Servo.clean_up()

with open('measurement.csv', 'w') as measurement_file:
    writer = csv.writer(measurement_file, lineterminator='\n')
    writer.writerow(fieldnames)
    writer.writerows(experiment_log)
