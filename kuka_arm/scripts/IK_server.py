#!/usr/bin/env python
# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.
# Author: Harsh Pandya
# import modules

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import math
import numpy as np

def handle_calculate_IK(req):

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    else:

        # Initialize service response

        joint_trajectory_list = []


        # Define DH param symbols - Parameters (dictionary)
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #length offsets
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #length links
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angles

        # Joint angle symbols - Variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i - Joint Angles Variables

            
        # Define Modified DH Transformation matrix
        s = {alpha0:     0,    a0:       0,     d1:    0.75,
             alpha1: -pi/2,    a1:    0.35,     d2:       0,   q2:  q2-pi/2,  
             alpha2:     0,    a2:    1.25,     d3:       0,     
             alpha3: -pi/2,    a3:  -0.054,     d4:    1.50,
             alpha4:  pi/2,    a4:       0,     d5:       0, 
             alpha5: -pi/2,    a5:       0,     d6:       0,
             alpha6:     0,    a6:       0,     d7:   0.303,   q7: 0}

        # Conversion Factors
        rtd = 180./np.pi # radians to degrees
        dtr = np.pi/180. # degrees to radians
        a12 = 0.4500 # meters
        a23 = 0.3000 # meters

        # Define Homogeneous individual transformation matrices- between Links ie base to Link
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)
            
        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)
        
        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)
        
        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)
        
        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)
        
        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)
        
        #Gripper Conversion     
        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_G = T6_G.subs(s)
        
        # Transform from base link to end effector - Gripper
        T0_2 = trigsimp(T0_1 * T1_2) # Base link to link 1 To simplify expressions using trigonometric (radian) identities, use trigsimp().
        T0_3 = trigsimp(T0_2 * T2_3) # Base to 3
        T0_4 = trigsimp(T0_3 * T3_4) # Base to 4
        T0_5 = trigsimp(T0_4 * T4_5) # Base to 5
        T0_6 = trigsimp(T0_5 * T5_6) # Base to 6
        T0_G = trigsimp(T0_6 * T6_G) # Base to Gripper  
        
        #Correction for orientation difference between defintion of gripper link URDF verse DH parameters
        # Preform the rotate around z-axis by pi first
        R_z = Matrix([[ cos(np.pi), -sin(np.pi),       0,      0],
                      [ sin(np.pi),  cos(np.pi),       0,      0],
                      [          0,           0,       1,      0],
                      [          0,           0,       0,      1]])
        #Second preform the rotate around y-axis by -pi/2
        R_y = Matrix([[ cos(-np.pi/2),        0,  sin(-np.pi/2),   0],
                      [             0,        1,              0,   0],
                      [-sin(-np.pi/2),        0,  cos(-np.pi/2),   0],
                      [             0,        0,              0,   1]])
            
        #calculate total correction factor
        R_corr = trigsimp(R_z * R_y)
        print "R_corr : %s " % R_corr # screen print to ensure scritpt is running.
        #calculate corrected transform from base to end effector
        #T_total = simplify(T0_G * R_corr)
        print "T_total : %s " % T_total # screen print to ensure scritpt is running.
        
        # Next extract the rotational component from the transformation matrix
        R0_3 = T0_3[0:3, 0:3]
            
            
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
     
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
     
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                   req.poses[x].orientation.z, req.poses[x].orientation.w])
         
            #Calculate joint angles using Geometric IK method
            #urdf values - Some values may not be needed but is complete list. XYZ
            j0_1_list = [0.000, 0, 0.330] # using rosrun tf tf_echo base_link link_1
            j0_2_list = [0.350, 0, 0.750] # using rosrun tf tf_echo base_link link_2
            j0_3_list = [0.350, 0, 2.000] # using rosrun tf tf_echo base_link link_3
            j0_4_list = [1.310, 0, 1.946] # using rosrun tf tf_echo base_link link_4
            j0_5_list = [1.850, 0, 1.946] # using rosrun tf tf_echo base_link link_5
            j0_6_list = [2.043, 0, 1.946] # using rosrun tf tf_echo base_link link_3
            j0_7_list = [1.850, 0, 1.946] # using rosrun tf tf_echo base_link link_5
            Gppr_List = [2.153, 0, 1.946] # using rosrun tf tf_echo base_link gripper_link
            l230 = 1.25
                                
            #Construct R0_6 based on target roll, pitch and yaw angles of end-effector 
            R_roll = Matrix([[ 1,                0,          0],
                             [ 0,        cos(roll), -sin(roll)],
                             [ 0,        sin(roll),  cos(roll)]])
     
            R_pitch = Matrix([[ cos(pitch),        0,  sin(pitch)],
                              [          0,        1,           0],
                              [-sin(pitch),        0,  cos(pitch)]])
     
            R_yaw = Matrix([[ cos(yaw), -sin(yaw),        0],
                            [ sin(yaw),  cos(yaw),        0],
                            [        0,         0,        1]])
     
            R0_6 = trigsimp(R_roll * R_pitch * R_yaw) # Compute the overall transformation 
     
            #Calculation for the Wrist Center 
            P_xyz = Matrix([[px],[py],[pz]])
            w_ctr = trigsimp(P_xyz - 0.303 * R0_6 * Matrix([[1],[0],[0]])) #wrist center
            j5 = w_ctr # matrix used for more calculations
                
            #Pre-calculations
            l3_5_X = j0_5_list[0] - j0_3_list[0]
            l3_5_Z = j0_5_list[2] - j0_3_list[2]
            l3_5 = sqrt(l3_5_X**2 + l3_5_Z**2)
                
            #Theta1  
            theta1 = atan2(j5[1], j5[0])
   		                
            #Pre-calculations
            j2 = [j0_2_list[0] * cos(theta1), j0_2_list[0] * sin(theta1), j0_2_list[2]]
            l2_5_X = j5[0] - j2[0]
            l2_5_Y = j5[1] - j2[1]
            l2_5_Z = j5[2] - j2[2]
            l2_5 = sqrt(l2_5_X**2 + l2_5_Y**2 + l2_5_Z**2)
            disp = (l2_5**2 - l230**2 - l3_5**2) / -(2 * l230 * l3_5)
            angle_3 = atan2(sqrt(1-disp**2), disp) 
                
            #Pre-Calculate to theta2
            m = l3_5 * sin(angle_3)
            m2 = l230 - l3_5 * cos(angle_3)
            b = atan2(j5[2]-j2[2], sqrt((j5[0]-j2[0])**2 + (j5[1]-j2[1])**2)) #Return atan(), in radians.
            b2 = atan2(m,m2)
            
            #Theta2    
            theta2 = pi/2-b2-b
                
            #Theta3  
            theta3 = pi/2-(atan2(sqrt(1-disp**2), disp) - atan2(l3_5_Z, l3_5_X))
                
            #Pre-calculation for Gripper Joint in theta's 4,5,and 6   
            R0_3_num = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            R0_3_num_inv = R0_3_num ** -1
            R3_6 = R0_3_num_inv * R0_6
                
            #Theta 4
            theta4 = atan2(R3_6[2,1],R3_6[2,2]) # rotation about X-axis
                    
            #Theta 5
            #theta5 = atan2(sqrt((R3_6[0,0])**2+ (R3_6[0,0])**2),R3_6[1,0])
            theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]*R3_6[0,0]+R3_6[1,0]*R3_6[1,0])) # rotation about Y-axis
                
            #Theta 6
            theta6 = atan2(R3_6[1,0],R3_6[0,0]) # rotation about Z-axis
            
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            print "Theta1 %s:" % theta1 #screen print to see the values 
	    print "Theta2 %s:" % theta2
            print "Theta3 %s:" % theta3
            print "Theta4 %s:" % theta4
            print "Theta5 %s:" % theta5
            print "Theta6 %s:" % theta6
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():

    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()



if __name__ == "__main__":
    IK_server()
