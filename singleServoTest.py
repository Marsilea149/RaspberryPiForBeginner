import RPi.GPIO as GPIO
import time 

GPIO.setmode(GPIO.BOARD)
# Set pin 12 as an output
GPIO.setup(12, GPIO.OUT)
# Set pin 11 as PWM with 50Hz pulse
servo1 = GPIO.PWM(12, 50)

# Start PWM with pulse off (value = 0)
servo1.start(0)
print("Waiting for 2 seconds")
time.sleep(2)

# Move the servo
print("Rotating 180 degrees in 10 steps")

# Define variable for duty cycle
duty = 2

# Loop through ducty cycle values between 2 and 12 (0 to 180 degrees)
while duty <= 12:
    servo1.ChangeDutyCycle(duty)
    time.sleep(1)
    duty = duty + 1

# Wait for 2 seconds
time.sleep(2)

# Clean up
servo1.stop()
GPIO.cleanup()
print("Goodbye")