"""
Probably want to try plugging rc car into computer & moving code to it via VSCode or Thonny (IDE)
Adjust the pin numbers here & see if it moves the car
"""

import RPi.GPIO as GPIO
import time



# Motor 1 GPIO pins - ADJUST
ENA = 17  # PWM pin for speed control
IN1 = 27
IN2 = 22

# Motor 2 GPIO pins - ADJUST
ENB = 18  # PWM pin for speed control
IN3 = 23
IN4 = 24

# Set up GPIO mode and warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up motor pins as output
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Initialize PWM instances
pwma = GPIO.PWM(ENA, 100)
pwmb = GPIO.PWM(ENB, 100)

# Initialize PWM (initial speed = 0)
pwma.start(0)
pwmb.start(0)

def forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def left(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def right(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)

try:
    while True:
        forward(50)  # Forward at 50% speed
        time.sleep(2)
        stop()
        time.sleep(1)

        backward(50)  # Backward at 50% speed
        time.sleep(2)
        stop()
        time.sleep(1)

        left(50)  # Left turn at 50% speed
        time.sleep(2)
        stop()
        time.sleep(1)

        right(50)  # Right turn at 50% speed
        time.sleep(2)
        stop()
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
