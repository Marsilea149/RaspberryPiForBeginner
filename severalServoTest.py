# Import libraries
import RPi.GPIO as GPIO
import time

# Define pwm frequency in Hz
FREQ = 50

# Creat servo pins variables
PIN_A = 13
PIN_B = 15
PIN_C = 16
PIN_D = 18

#Convert an input angle in degree to percentage of duty cycle
def angle_to_dc(angle):
    return angle / 18 + 2;
    
# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set servo pins as an output
GPIO.setup(PIN_A, GPIO.OUT)
GPIO.setup(PIN_B, GPIO.OUT)
GPIO.setup(PIN_C, GPIO.OUT)
GPIO.setup(PIN_D, GPIO.OUT)

# Define servo pins as PWM pin
servo_A = GPIO.PWM(PIN_A, FREQ)
servo_B = GPIO.PWM(PIN_B, FREQ)
servo_C = GPIO.PWM(PIN_C, FREQ)
servo_D = GPIO.PWM(PIN_D, FREQ) 


# Start PWM running, with value of 0 (pulse off)
servo_A.start(0)
servo_B.start(0)
servo_C.start(0)
servo_D.start(0)

# Loop to allow user to set servo angle. Try/finally allows exit
# with execution of servo.stop and GPIO cleanup :)

try:
    while True:
        #Ask user for angle and turn servo to it
        angle = float(input('Enter angle between 0 & 180: '))
        servo_A.ChangeDutyCycle(angle_to_dc(angle))
        servo_B.ChangeDutyCycle(angle_to_dc(angle))
        servo_C.ChangeDutyCycle(angle_to_dc(angle))
        servo_D.ChangeDutyCycle(angle_to_dc(angle))
        time.sleep(0.5)
        servo_A.ChangeDutyCycle(0)
        servo_B.ChangeDutyCycle(0)
        servo_C.ChangeDutyCycle(0)
        servo_D.ChangeDutyCycle(0)

finally:
    #Clean things up at the end
    servo_A.stop()
    servo_B.stop()
    servo_C.stop()
    servo_D.stop()
    GPIO.cleanup()
    print("Goodbye!")

