#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import fkArmRobot as fk

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
    #input joint angles in joint frame in degree
    #range: -80 (robot's right)=>80 (robot's left)
    base_angle = 0
    #range: 70(up)=>-30(down)
    shoulder_angle = 70
    #range: 60(up)=>-40(down)
    elbow_angle = -40
    
    #Define the neutral angles for each joint
    #At these angle the robot arm is athe the axis x for each joint
    base_neutral = 90
    shoulder_neutral = 115
    elbow_neutral = 110
    
    # Take into account the servo offset with 'neutral' values of each servo 
    #base: 10->170  neutral=90
    pca.servo[0].angle = base_angle + base_neutral
    #shoulder: 45->145  neutral=115
    pca.servo[1].angle = shoulder_neutral - shoulder_angle 
    #elbow: 60->150   neutral=110
    pca.servo[2].angle = elbow_neutral - elbow_angle
    #gripper:85=open 122=close
    pca.servo[3].angle = 122
    
    fk.compute_end_effector_pose(base_angle, shoulder_angle, elbow_angle)
    
    
    
