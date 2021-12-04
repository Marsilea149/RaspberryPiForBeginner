# Import libraries
import RPi.GPIO as GPIO
import time

# Servo pin
sPin = 12

#Convert an input angle in degree to percentage of duty cycle
def angle_to_dc(angle):
    return angle / 18 + 2;
    
# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and define as servo1 as PWM pin
GPIO.setup(sPin,GPIO.OUT)
servo1 = GPIO.PWM(sPin, 50) # pulse 50Hz

# Start PWM running, with value of 0 (pulse off)
servo1.start(0)

# Loop to allow user to set servo angle. Try/finally allows exit
# with execution of servo.stop and GPIO cleanup :)

try:
    while True:
        #Ask user for angle and turn servo to it
        angle = float(input('Enter angle between 0 & 180: '))
        servo1.ChangeDutyCycle(angle_to_dc(angle))
        time.sleep(0.5)
        servo1.ChangeDutyCycle(0)

finally:
    #Clean things up at the end
    servo1.stop()
    GPIO.cleanup()
    print("Goodbye!")

