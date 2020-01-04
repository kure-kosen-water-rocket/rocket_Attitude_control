import time
import pigpio
from pigpio import pi


class ServoController:
    def __init__(self, Pin):
        self.pi = pi()
        self.mPin = Pin
        self.pi.set_mode(self.mPin, pigpio.OUTPUT)

    def set_position(self, degree):
        duty = int(((12-2.5)/180*degree+2.5)*10000) #issue番号(#2)を参照
        self.pi.hardware_PWM(self.mPin, 50, duty)

    def cleanup_gpio(self):
        self.pi.hardware_PWM(self.mPin, 50, 0)
        time.sleep(1)
        self.pi.set_mode(self.mPin, pigpio.INPUT)
        self.pi.stop()