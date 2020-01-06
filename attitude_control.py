import math
import time
import csv
from servo import ServoController
from mpu6050 import Mpu6050


#初期値の設定
dt = 0
total_time = 0
old_y_gyro = 0
old_angle = 0
fieldnames = ['x_accel', 'y_accel', 'z_accel', 'x_gyro', 'y_gyro', 'z_gyro', 'dt']
data_logs = []
CALC_TIME = 20

mpu6050 = Mpu6050()

Servo = ServoController(Pin=18)
Servo.set_position(90) #初期位置を90度に設定

while 1:
    start = time.time() #ループ中にかかる時間を計測開始

    x_gyro,  y_gyro,  z_gyro  = mpu6050.gyro_lsb()
    x_accel, y_accel, z_accel = mpu6050.accel_lsb()

    data_logs.append([x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro, dt])

    y_angle = math.atan2(x_accel , math.sqrt(y_accel**2 + z_accel**2))#姿勢角の算出
    k = 0.2 * (65536**-(abs(x_accel)/10)) #加速度が増加→加速度から得られる角度の比重を小さくしていく係数k
    y_angle = (((1-k) * (old_angle + math.radians(((old_y_gyro + y_gyro)* dt) / 2)) + k* y_angle)) #加速度とジャイロの相補フィルター
    old_angle = y_angle #角度の変化量を足していくため一つ前の角度が必要
    old_y_gyro = y_gyro #台形積分用に一つ前の角速度を残しておく

    adjust_y_angle = 90-int(math.degrees(y_angle)) #初期位置は90度からなので

    if adjust_y_angle < 180 and adjust_y_angle > 0: #センサーが0度~180度の時のみ動作
        Servo.set_position(adjust_y_angle)

    #計測時間がtotal_timeに達したらプログラム終了
    if CALC_TIME < int(total_time):
        break

    dt = time.time() - start #ループにかかる時間を微小時間dtに代入
    total_time += dt

Servo.clean_up()

with open('measurement.csv', 'w') as measurement_file:
    writer = csv.writer(measurement_file, lineterminator='\n')
    writer.writerow(fieldnames)
    writer.writerows(data_logs)
