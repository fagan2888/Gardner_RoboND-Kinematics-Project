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
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        passes = 0
        for x in xrange(0, len(req.poses)):
            passes = passes + 1  
            print "Currently processing ",passes,"of %s Poses." % len(req.poses)          
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6=symbols('alpha0:7')
            a0,a1,a2,a3,a4,a5,a6=symbols('a0:7')
            # Joint angle symbols
            q1,q2,q3,q4,q5,q6,q7= symbols('q1:8')
            d1,d2,d3,d4,d5,d6,d7= symbols('d1:8') 
            # Modified DH params
            #Defining constants
            S={alpha0:    0, a0:     0, d1: 0.75,
               alpha1:-pi/2, a1:  0.35, d2:    0,  q2:q2-pi/2,
               alpha2:    0, a2:  1.25, d3:    0,
               alpha3:-pi/2, a3:-0.054, d4:  1.5,
               alpha4: pi/2, a4:     0, d5:    0,
               alpha5:-pi/2, a5:     0, d6:    0,
               alpha6:    0, a6:     0, d7:0.303,   q7:     0}

            # Create individual transformation matrices
            T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                           [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                           [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                           [                   0,                   0,            0,               1]])
            T0_1 = T0_1.subs(S)
            
            T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                           [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                           [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                           [                   0,                   0,            0,               1]])
            T1_2 = T1_2.subs(S)
        
            T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                           [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                           [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                           [                   0,                   0,            0,               1]])
            T2_3 = T2_3.subs(S)
        
            T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                           [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                           [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                           [                   0,                   0,            0,               1]])
            T3_4 = T3_4.subs(S)
        
            T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                           [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                           [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                           [                   0,                   0,            0,               1]])
            T4_5 = T4_5.subs(S)
        
            T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                           [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                           [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                           [                   0,                   0,            0,               1]])
            T5_6 = T5_6.subs(S)
        
            #Gripper Conversion     
            T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                           [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                           [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                           [                   0,                   0,            0,               1]])
            T6_G = T6_G.subs(S)

            #Correction for orientation difference between defintion of gripper link URDF verse DH parameters
            # Preform the rotate around z-axis by pi first            
            #Correcting gripper cordinate by 180 on Z-axis and 90 on y-axis
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
            
            #Calculate total correction factor
            R_corr = trigsimp(R_z * R_y)

            # Transform from base link to end effector - Gripper
            T0_2 = trigsimp(T0_1 * T1_2) # Base link to link 1 To simplify expressions using trigonometric (radian) identities, use trigsimp().
            T0_3 = trigsimp(T0_2 * T2_3) # Base to 3
            T0_4 = trigsimp(T0_3 * T3_4) # Base to 4
            T0_5 = trigsimp(T0_4 * T4_5) # Base to 5
            T0_6 = trigsimp(T0_5 * T5_6) # Base to 6
            T0_G = trigsimp(T0_6 * T6_G) # Base to Gripper
            T0_G_Orient=trigsimp(T0_G*R_corr) # Gripper orientation correction
            
        
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            P_xyz=Matrix([[px],[py],[pz],[0]])
            D_offset=Matrix([[d7],[0],[0],[0]])

            #Euler angle from Quaternion Conversion
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                   req.poses[x].orientation.z, req.poses[x].orientation.w])
        
            print '****End Effector Information***'
            print 'Attribute - Value'
            print '    Roll  : ',roll
            print '    Pitch : ',pitch
            print '    PX    : ',px
            print '    PY    : ',py
            print '    PZ    : ',pz
            print '*******************************'


            # Calculate joint angles using Geometric IK method
            R_roll = Matrix([[ 1,          0,          0,    0],
                             [ 0,  cos(roll), -sin(roll),    0],
                             [ 0,  sin(roll),  cos(roll),    0],
		    	             [ 0,          0,          0,    1]]) 

            R_pitch = Matrix([[ cos(pitch),  0,  sin(pitch), 0],
                              [          0,  1,           0, 0],
                              [-sin(pitch),  0,  cos(pitch), 0],
			                  [          0,  0,           0, 1]])
     
            R_yaw = Matrix([[ cos(yaw), -sin(yaw),   0,   0],
                            [ sin(yaw),  cos(yaw),   0,   0],
                            [        0,         0,   1,   0],
	                        [        0,         0,   0,   1]])
     
            R0_6 = trigsimp(R_roll * R_pitch * R_yaw) # Compute the overall transformation 

            #Wrist computation
            Wrist=P_xyz-R0_6*D_offset
            Wrist_Cntr=Wrist.subs(S)
            WC_x=Wrist_Cntr[0]
            WC_y=Wrist_Cntr[1]
            WC_z=Wrist_Cntr[2]

            #Print Wrist Center Values
            print ''
            print '****WRIST CENTER POSITION****'
            print 'Wrist Center x : ',WC_x
            print 'Wrist Center y : ',WC_y
            print 'Wrist Center z : ',WC_z
            print '*****************************'
           
           
            #Theta calculation - #1
            theta1=atan2(WC_y,WC_x)
            
            #Theta calculation - #2 
            r=sqrt(WC_x*WC_x+WC_y*WC_y)-a1
            r=r.subs(S)
            s=WC_z-d1
            s=s.subs(S)
            h=sqrt(r*r+s*s)
            a_2=atan2(s,r)
            d3_4=sqrt(d4*d4+a3*a3)
            d3_4=d3_4.subs(S)
            lngth=((a2*a2+h*h-d3_4*d3_4)/(2*h*a2))
            lngth=lngth.subs(S)
            b_2=acos(lngth)
            b_2.subs(S)
            theta2=(np.pi/2-(abs(a_2)+b_2))
            
            #Theta calculation - #3
            tc_3=((a2*a2+d3_4*d3_4-h*h)/(2*d3_4*a2))
            tc_3.subs(S)
            a_3=acos(tc_3)
            a_3=a_3.subs(S)
            b_3=atan2(d4,a3)
            b_3=b_3.subs(S)
            theta3=(+np.pi-(a_3+b_3))
            theta3=theta3.subs(S)
            
            #R3_6 calculation
            T0_3=T0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
            R0_3=T0_3[:3,:3]
            R3_6=(R0_3.inv())*R0_6[:3,:3]
            R3_6_numpy=np.array(R3_6).astype(np.float64)
            alpha,beta,gamma=tf.transformations.euler_from_matrix(R3_6_numpy,axes='ryzx')
            #Theta calculation - #4-6
            theta4=alpha
            theta5=(beta-np.pi/2)
            theta6=(gamma-np.pi/2)
            #Quick Adjust for 0 on theta 5 
            if theta5==0:
               theta4=0
               theta6=0

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            
        #Print Theta Vaues
        print '**'
        print '*****************'
        print 'THETA 1 :',theta1
        print 'THETA 2 :',theta2
        print 'THETA 3 :',theta3
        print 'THETA 4 :',theta4
        print 'THETA 5 :',theta5
        print 'THETA 6 :',theta6
        print '*****************'
            
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
