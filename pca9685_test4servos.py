#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

#Constants
nbPCAServo=4

#Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

#Objects
pca = ServoKit(channels=16)
pca.frequency = 50

# function init 
def init():

    for i in range(nbPCAServo):
        pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])


# function main 
def main():

    pcaScenario();


# function pcaScenario 
def pcaScenario():
    """
    #Scenario to test servo: test servos one by one
    for i in range(nbPCAServo):
        for j in range(MIN_ANG[i],MAX_ANG[i],1):
            print("Send angle {} to Servo {}".format(j,i))
            pca.servo[i].angle = j
            time.sleep(0.01)
            
        for j in range(MAX_ANG[i],MIN_ANG[i],-1):
            print("Send angle {} to Servo {}".format(j,i))
            pca.servo[i].angle = j
            time.sleep(0.01)
            
        pca.servo[i].angle=None #disable channel
        time.sleep(0.5)
    """
    #Scenario to test servo: test servos all same time
    for j in range(0, 180, 1):
        print("Send angle {} to all Servos".format(j))
        pca.servo[0].angle = j
        pca.servo[1].angle = j
        pca.servo[2].angle = j
        pca.servo[3].angle = j
        time.sleep(0.01)
        
    for j in range(180, 0, -1):
        print("Send angle {} to all Servos".format(j))
        pca.servo[0].angle = j
        pca.servo[1].angle = j
        pca.servo[2].angle = j
        pca.servo[3].angle = j
        time.sleep(0.01)

if __name__ == '__main__':
    init()
    main()
