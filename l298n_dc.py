import RPi.GPIO as GPIO          
from time import sleep
import os



class Motor:

    # Initialising and setting up the board
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

    # Setting the motor to move forward
    def motorf(self,time=10, freq=20):
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW)
        self.pwm.ChangeDutyCycle(freq)
        sleep(time)
    
    # Setting the motor to move backward
    def motorb(self,time=10, freq=20):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH)
        self.pwm.ChangeDutyCycle(freq)
        sleep(time)
    
    # Setting the motor to stop
    def braking(self,time=2):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        sleep(time)


motor = Motor(16,18,22)

# Creating an infinite loop to investigate the motor movement unless interrupted
try:
    while True:
        motor.motorf()
        motor.braking()
        motor.motorb()

except KeyboardInterrupt:
    os.sys.exit(0)

# Garbage collecting 
finally:
    GPIO.cleanup()
