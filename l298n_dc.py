import RPi.GPIO as GPIO          
from time import sleep
import os



class Motor:
    def __init__(self,in1,in2,ena):
        self.in1 = in1
        self.in2 = in2
        self.ena = ena
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(in1,GPIO.OUT)
        GPIO.setup(in2,GPIO.OUT)
        GPIO.setup(ena,GPIO.OUT)
        self.pwm=GPIO.PWM(self.ena,1000)
        self.pwm.start(0)
        
    def motorf(self,time=0, freq=20):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH)
        self.pwm.ChangeDutyCycle(freq)
        sleep(time)
    
    def motorb(self,time=0, freq=20):
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW)
        self.pwm.ChangeDutyCycle(freq)
        sleep(time)

    def braking(self,time=2):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        sleep(time)

motor = Motor(16,18,22)
try:
    while True:
        motor.motorf()
        motor.braking()
        motor.motorb()

except KeyboardInterrupt:
    os.sys.exit(0)

finally:
    GPIO.cleanup()
