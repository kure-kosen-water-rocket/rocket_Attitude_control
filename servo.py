import time
import pigpio


class ServoController:
    def __init__(self, Pin): #BCM番号ではなく物理的なpin番号
        self.pi = pigpio.pi()
        self.mPin = Pin
        self.pi.set_mode(self.mPin, pigpio.OUTPUT)

    def set_position(self, degree):
         # ref: https://github.com/kure-kosen-water-rocket/rocket_attitude_control/issues/2
        duty = int(((12-2.5)/180*degree+2.5)*10000)
        self.pi.hardware_PWM(self.mPin, 50, duty)

    def clean_up(self):
        self.pi.hardware_PWM(self.mPin, 50, 0)
        time.sleep(1)
        self.pi.set_mode(self.mPin, pigpio.INPUT)
        self.pi.stop()
