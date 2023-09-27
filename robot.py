#This is a code for the line tracking robot robot that uses servo motor for steering and a DC motor for forward and reverse, it contains two ir sensors

import RPi.GPIO as GPIO
import time

# Set GPIO mode and pin numbers
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

ena_motor_pin = 3
in1_motor_pin = 5 
in2_motor_pin = 7 # Replace with the actual pin numbers for your motors
servo_pin = 8 # Replace with the actual pin number for your servo

# Set up GPIO pins
GPIO.setup(ena_motor_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(in1_motor_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(in2_motor_pin, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(servo_pin, GPIO.OUT)

# Set up PWM for servo motor
pwm = GPIO.PWM(servo_pin, 50)# 50 Hz PWM frequency
pwm.start(7.5)# Center the servo initially (adjust as needed)
# Line tracking sensor setup (replace with your actual sensor setup)
left_sensor_pin = 10
right_sensor_pin = 12

GPIO.setup(left_sensor_pin, GPIO.IN)
GPIO.setup(right_sensor_pin, GPIO.IN)
# Function to control the motors based on sensor readings
def line_tracking():
    while True:
        left_sensor_value = GPIO.input(left_sensor_pin)
        right_sensor_value = GPIO.input(right_sensor_pin)
        print("Left_sensor_value:")
        print(left_sensor_value)
        print("Right_sensor_value:")
        print(right_sensor_value)
        # Implement bang-bang control logic
        if left_sensor_value == 0 and right_sensor_value == 0:
            # Both sensors are on the line, move forward
            GPIO.output(ena_motor_pin, GPIO.HIGH)
            GPIO.output(in1_motor_pin, GPIO.HIGH)
            GPIO.output(in2_motor_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(8.0)# 90 degrees
            time.sleep(1)
        elif left_sensor_value == 1 and right_sensor_value == 0:
            # Left sensor is off the line, turn right
            GPIO.output(ena_motor_pin, GPIO.HIGH)
            GPIO.output(in1_motor_pin, GPIO.HIGH)
            GPIO.output(in2_motor_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(11.0)# 90 degrees
            time.sleep(1)
        elif left_sensor_value == 0 and right_sensor_value == 1:
            # Right sensor is off the line, turn left
            GPIO.output(ena_motor_pin, GPIO.HIGH)
            GPIO.output(in1_motor_pin, GPIO.HIGH)
            GPIO.output(in2_motor_pin, GPIO.LOW)
            # Rotate the servo to 0 degrees (change as needed)
            pwm.ChangeDutyCycle(6.0)# 0 degrees
            time.sleep(1)
        else:
            # Both sensors are off the line or on the line, stop
            GPIO.output(ena_motor_pin, GPIO.LOW)
            GPIO.output(in1_motor_pin, GPIO.LOW)
            GPIO.output(in2_motor_pin, GPIO.LOW)
# Run the line tracking function
try:
    line_tracking()
except KeyboardInterrupt:
    pass
# Clean up GPIO pins
GPIO.cleanup()
