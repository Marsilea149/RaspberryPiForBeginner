# -*- coding: utf-8 -*-
"""
This file implements inverse kinematics using gradient descent 
Created on Sun Dec 12 15:45:29 2021

@author: Marsi
"""
import numpy as np
import fkArmRobot as fk

# Global variables
#SamplingDistance for joint angles in degree
SamplingDistance = 0.01
#LearningRate for gradient descent
LearningRate = 2
#DistanceThreshold used to stop iteration process for gradient descent
DistanceThreshold = 0.1

#minimum constraints of the joint angles in degree
MinAngles = [-30, -30, -30]
#maximum constraints of the joint angles in degree
MaxAngles = [30, 30, 30]

"""
This method computes the distance from the target point to the current end effector pose
@param target: a vector of (3x1), target end effector pose in global frame
@param joint_angles: a vector of (3x1), [base_angle, shoulder_angle, elbow_angle] in degree
"""
def distance_from_target(target, joint_angles):
    base_angle = joint_angles[0]
    shoulder_angle = joint_angles[1]
    elbow_angle = joint_angles[2]
    current_eff_pos = fk.compute_end_effector_pose(base_angle, shoulder_angle, elbow_angle)
    #compute the error vector
    delta_p = target - current_eff_pos
    return numpy.linalg.norm(delta_p)

"""
This method computes the partial derivative of the error distance wrt i-th joint variable.
Here we use a samplingDistance, representing a small change of joint angle, to achieve partial gradidient calculation.
@param target: a vector of (3x1), target end effector pose in global frame
@param joint_angles: a vector of (3x1), [base_angle, shoulder_angle, elbow_angle] in degree
@param i: i-th joint 

"""
def partial_gradient(target, joint_angles, i):
    #save the angle, this will be restored later
    angle =  joint_angles[i]
    
    #compute gradient: [F(x+samplingDistance) - F(x)] / samplingDistance
    #compute F(x)
    f_x = distance_from_target(target, joint_angles)
    #compute F(x+samplingDistance)
    joint_angles[i] += SamplingDistance
    f_x_plus_d = distance_from_target(target, joint_angles)
    #compute gradient
    gradient = (f_x_plus_d - f_x)/SamplingDistance
    
    #Restore angle
    joint_angles[i] = angle
    
    return gradient


"""
This method compues the joint_angles to achieve inverse kinematics
@param target: a vector of (3x1), target end effector pose in global frame
@param joint_angles: a vector of (3x1), [base_angle, shoulder_angle, elbow_angle] in degree
"""
def ik(target, joint_angles):
    n = len(joint_angles)
    if distance_from_target(target, joint_angles) < DistanceThreshold:
        return;
    print("target: ")
    print(target)
    for i in range(n):
        gradient = partial_gradient(target, joint_angles, i)
        #compute solution: joint_angle = joint_angle - LearningRate*Gradient
        joint_angles[i] -= LearningRate * gradient 
        #Limit the raneg of motion
        joint_angles[i] = np.clip(joint_angles[i], MinAngles[i], MaxAngles[i])
        #Early termination
        if distance_from_target(target, joint_angles) < DistanceThreshold:
            return;
        print("joint_angles_cmd[", i, "]: ", joint_angles[i])
        