#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

#Constants used to set the nomber of servos in use
nbPCAServo=4

#Parameters used to map between pwm pulse (ms) and servo angles (degree)
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

#Objects
pca = ServoKit(channels=16)
pca.frequency = 50

# initialize the servos
def init():
    for i in range(nbPCAServo):
        pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])


# move the servo with input angles  
def servo_control(ang1, ang2, ang3, ang4):
    print("Send angle {} to Servo1".format(ang1))
    pca.servo[0].angle = ang1
    print("Send angle {} to Servo2".format(ang2))
    pca.servo[1].angle = ang2
    print("Send angle {} to Servo3".format(ang3))
    pca.servo[2].angle = ang3
    print("Send angle {} to Servo4".format(ang4))
    pca.servo[3].angle = ang4
    time.sleep(0.01)



if __name__ == '__main__':
    init()
    servo_control(0, 0, 0, 0)
    time.sleep(2)
    servo_control(180, 90, 150, 160)
    time.sleep(2)
    servo_control(0, 0, 0, 0)
    time.sleep(2)

