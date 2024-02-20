"""
Goal is to continuously calculate angle of movement via camera & math. 
A better method probably exists. Untested.
Alter pin #s as necessary
"""

import RPi.GPIO as GPIO
import time

# Rotary encoder GPIO pins
encoder_dt = 13
encoder_clk = 19

# Initialize GPIO mode and warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up encoder pins as input with pull-up resistors
GPIO.setup(encoder_dt, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_clk, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize variables
encoderPos = 0
direction = 0

# Function to update encoder position
def encoder_callback(channel):
    global encoderPos
    global direction
    
    dt_state = GPIO.input(encoder_dt)
    clk_state = GPIO.input(encoder_clk)
    
    if dt_state == 1 and clk_state == 0:
        direction = 1  # Clockwise rotation
    elif dt_state == 0 and clk_state == 1:
        direction = -1  # Counter-clockwise rotation
    
    encoderPos += direction

# Add event detection to the encoder pins
GPIO.add_event_detect(encoder_dt, GPIO.BOTH, callback=encoder_callback)
GPIO.add_event_detect(encoder_clk, GPIO.BOTH, callback=encoder_callback)

try:
    while True:
        # Read the current angle of travel
        current_angle = encoderPos  # You may need to scale this value based on your encoder setup
        
        # Print the current angle
        print("Current angle of travel:", current_angle)
        
        # Adjust the loop delay according to your requirements
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
