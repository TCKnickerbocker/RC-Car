"""
Goal is to drive using camera as detection. Untested.
Alter pin #s as necessary
"""

import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Motor GPIO pins
ENA = 17  # PWM pin for left motor speed control
IN1 = 27
IN2 = 22

ENB = 18  # PWM pin for right motor speed control
IN3 = 23
IN4 = 24

# Ultrasonic sensor GPIO pins
TRIG = 5
ECHO = 6

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

# Set up ultrasonic sensor pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Initialize PWM instances
pwma = GPIO.PWM(ENA, 100)
pwmb = GPIO.PWM(ENB, 100)

# Initialize PWM (initial speed = 0)
pwma.start(0)
pwmb.start(0)

# Function to process image and detect white lines
def detect_white_lines(image):
    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Threshold the image to get binary image
    _, threshold = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
    
    # Find contours in the binary image
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Iterate through contours to find white lines
    white_lines = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:  # Adjust threshold according to your environment
            x, y, w, h = cv2.boundingRect(contour)
            white_lines.append((x, x + w))  # Store the x-coordinates of the white lines
    
    return white_lines

# Function to control the car movement based on the detected white lines and obstacle distance
def steer_car(white_lines, frame_width, obstacle_distance):
    # If obstacle is too close, stop the car
    if obstacle_distance < 10:  # Adjust threshold according to your setup
        return 0
    
    if len(white_lines) == 0:
        return 0  # If no white lines detected, go straight
    
    # Calculate the average position of white lines
    avg_x = sum([line[0] + (line[1] - line[0]) / 2 for line in white_lines]) / len(white_lines)
    
    # Calculate the deviation from the center
    deviation = avg_x - frame_width / 2
    
    # Define steering angle range (-1 to 1, where -1 means full left turn, 1 means full right turn)
    steering_range = 1
    
    # Calculate steering angle based on deviation from the center
    steering_angle = deviation / (frame_width / 2) * steering_range
    
    return steering_angle

# Function to measure distance from obstacle using ultrasonic sensor
def measure_distance():
    # Send a 10us pulse to trigger
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo response
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate pulse duration and convert to distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound is 343 m/s or 17150 cm/s

    return distance

# Initialize camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera

# Set the resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    # Measure distance from obstacle
    obstacle_distance = measure_distance()

    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Flip the frame horizontally (optional, depending on camera orientation)
    frame = cv2.flip(frame, 1)
    
    # Detect white lines
    white_lines = detect_white_lines(frame)
    
    # Calculate steering angle based on white lines and obstacle distance
    frame_width = frame.shape[1]
    steering_angle = steer_car(white_lines, frame_width, obstacle_distance)
    
    # Adjust the motor speed and direction based on steering angle
    speed = 50  # Set a constant speed for demonstration
    if steering_angle < 0:
        # Turn left
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwma.ChangeDutyCycle(speed)
        pwmb.ChangeDutyCycle(speed - abs(steering_angle * speed))
    elif steering_angle > 0:
        # Turn right
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwma.ChangeDutyCycle(speed - abs(steering_angle * speed))
        pwmb.ChangeDutyCycle(speed)
    else:
        # Go straight
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwma.ChangeDutyCycle(speed)
        pwmb.ChangeDutyCycle(speed)

    # Display the resulting frame
    cv2.imshow('Frame', frame)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
