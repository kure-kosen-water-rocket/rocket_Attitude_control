import math
import time
import csv
from servo import ServoController
from mpu6050 import Mpu6050


# 初期値の設定
dt = 0
current_time = 0
prev_gyro = 0
prev_angle = 0
fieldnames = ['accel_x', 'accel_y', 'accel_z',
              'gyro_x', 'gyro_y', 'gyro_y', 'dt']
instrumentation_logs = []
CALC_TIME = 50


def find_angle_in_acceleration(accel_x, accel_y, accel_z):
    accel_only_angle = math.atan2(
        accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2))
    return accel_only_angle


def complement_filter_coefficient(accel_x):
    factor = 0.2 * (65536 ** -(abs(accel_x) / 10))  # 加速度の大きさによって変化する係数
    return factor


def complement_filter(accel_only_angle, y_gyro, prev_angle, prev_gyro, factor):
    real_angle = (1 - factor) * (prev_angle + math.radians((prev_gyro +
                                                            y_gyro) * dt / 2)) + factor * accel_only_angle  # 相補フィルター
    prev_angle = real_angle  # 角度の変化量を足していくため一つ前の角度が必要
    prev_gyro = y_gyro  # 台形積分用に一つ前の角速度を残しておく
    adjust_angle = 90 - int(math.degrees(real_angle))  # -90度~90度の範囲を0度~180度に変換
    return [adjust_angle, prev_angle, prev_gyro]


mpu6050 = Mpu6050()

servo = ServoController(Pin=18)  # 物理ピン番号
servo.set_position(90)  # 初期位置を90度に設定

while 1:
    start = time.time()  # ループ中にかかる時間を計測開始

    gyro_x,  gyro_y,  gyro_z = mpu6050.gyro_lsb()
    accel_x, accel_y, accel_z = mpu6050.accel_lsb()

    instrumentation_logs.append(
        [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt])

    accel_only_angle = find_angle_in_acceleration(accel_x, accel_y, accel_z)

    factor = complement_filter_coefficient(accel_x)

    adjust_angle, prev_angle, prev_gyro = complement_filter(
        accel_only_angle, gyro_y, prev_angle, prev_gyro, factor)

    if adjust_angle < 180 and adjust_angle > 0:  # 0度~180度の時のみ動作
        servo.set_position(adjust_angle)

    # 計測時間がcurrent_timeに達したらプログラム終了
    if CALC_TIME < int(current_time):
        break

    dt = time.time() - start  # ループにかかる時間を微小時間dtに代入
    current_time += dt

servo.clean_up()

with open('measurement.csv', 'w') as measurement_file:
    writer = csv.writer(measurement_file, lineterminator='\n')
    writer.writerow(fieldnames)
    writer.writerows(instrumentation_logs)
