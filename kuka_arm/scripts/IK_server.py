#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    class Kuka_KR210:

        #symbols used


        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #angles of revolute joint
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        r, p, y = symbols("r p y")

        #DH Parameter Table
        s = {alpha0: 0,      a0: 0,     d1: 0.75,
             alpha1: -pi/2.,  a1: 0.35,  d2: 0,       q2: q2-pi/2.,
             alpha2: 0,      a2: 1.25,  d3: 0,
             alpha3: -pi/2.,  a3: -0.054,d4: 1.5,
             alpha4: pi/2.,   a4: 0,     d5: 0,
             alpha5: -pi/2.,  a5: 0,     d6: 0,
             alpha6: 0,      a6: 0,     d7: 0.303,  q7: 0}

        #known Transform Matricies for Kuka_KR210 with given DH Parameter Table
        T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0.750000000000000], [0, 0, 0, 1]])

        T1_2 = Matrix([[cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.350000000000000], [0, 0, 1, 0], [-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0], [0, 0, 0, 1]])

        T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25000000000000], [sin(q3), cos(q3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        T3_4 = Matrix([[cos(q4), -sin(q4), 0, -0.0540000000000000], [0, 0, 1, 1.50000000000000], [-sin(q4), -cos(q4), 0, 0], [0, 0, 0, 1]])

        T4_5 = Matrix([[cos(q5), -sin(q5), 0, 0], [0, 0, -1, 0], [sin(q5), cos(q5), 0, 0], [0, 0, 0, 1]])

        T5_6 = Matrix([[cos(q6), -sin(q6), 0, 0], [0, 0, 1, 0], [-sin(q6), -cos(q6), 0, 0], [0, 0, 0, 1]])

        T6_G = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.303000000000000], [0, 0, 0, 1]])

        T0_2 = Matrix([[sin(q2)*cos(q1), cos(q1)*cos(q2), -sin(q1), 0.35*cos(q1)], [sin(q1)*sin(q2), sin(q1)*cos(q2), cos(q1), 0.35*sin(q1)], [cos(q2), -sin(q2), 0, 0.750000000000000], [0, 0, 0, 1]])

        T0_3 = Matrix([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1), (1.25*sin(q2) + 0.35)*cos(q1)], [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3), cos(q1), (1.25*sin(q2) + 0.35)*sin(q1)], [cos(q2 + q3), -sin(q2 + q3), 0, 1.25*cos(q2) + 0.75], [0, 0, 0, 1]])

        T0_4 = Matrix([[sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4), sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1), -sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4), sin(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [cos(q4)*cos(q2 + q3), -sin(q4)*cos(q2 + q3), -sin(q2 + q3), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        T0_5 = Matrix([[(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), sin(q4)*cos(q2 + q3), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        T0_6 = Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        T0_G =  Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)], [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)], [-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        R_x = Matrix([  [1,         0,          0],
                        [0,         cos(r),     -sin(r)],
                        [0,        sin(r),      cos(r)]])

        R_y = Matrix([  [cos(p),    0,      sin(p)],
                        [0,         1,      0],
                        [-sin(p),   0,      cos(p)]])



        R_z = Matrix([  [cos(y),    -sin(y),    0],
                        [sin(y),    cos(y),     0],
                        [0,         0,          1]])


        R_G = Matrix([[cos(p)*cos(y), sin(p)*sin(r)*cos(y) - sin(y)*cos(r), sin(p)*cos(r)*cos(y) + sin(r)*sin(y)], [sin(y)*cos(p), sin(p)*sin(r)*sin(y) + cos(r)*cos(y), sin(p)*sin(y)*cos(r) - sin(r)*cos(y)], [-sin(p), sin(r)*cos(p), cos(p)*cos(r)]])

    Kuka = Kuka_KR210()

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols

    	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #angles of revolute joint
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        r, p, y = symbols("r p y")

    	# Create Modified DH parameters
    	# # Retrieve Modified DH parameters from Kuka_KR210 Class
        s = Kuka.s

    	# Define Modified DH Transformation matrix


    	# Create individual transformation matrices
        ##Assign Matrix Values from Kuka_KR210 Class
        T0_1 = Kuka.T0_1
        T1_2 = Kuka.T1_2
        T2_3 = Kuka.T2_3
        T3_4 = Kuka.T3_4
        T4_5 = Kuka.T4_5
        T5_6 = Kuka.T5_6
        T6_G = Kuka.T6_G
        T0_2 = Kuka.T0_2
        T0_3 = Kuka.T0_3
        T0_4 = Kuka.T0_4
        T0_5 = Kuka.T0_5
        T0_6 = Kuka.T0_6
        T0_G = Kuka.T0_G

    	# Extract rotation matrices from the transformation matrices
    	R_x = Kuka.R_x
        R_y = Kuka.R_y
        R_z = Kuka.R_z
        R_G_Base = Kuka.R_G

        # Compensate for rotation discrepancy between DH parameters and Gazebo
        R_Err = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
        R_G_Base = R_G_Base * R_Err


	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###


        # Initialize service response
        joint_trajectory_list = []

        theta1_debug = ' '
        theta2_debug = ' '
        theta3_debug = ' '
        theta4_debug = ' '
        theta5_debug = ' '
        theta6_debug = ' '
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

            ### Your IK code here

            R_G = R_G_Base.subs({'r': roll, 'p': pitch, 'y': yaw}) #substitute symbols in R_G

            #Gripper Position
            G_pos = Matrix([ [px], [py], [pz]]) #gripper position

            #Wrist Position
            W_pos = G_pos - 0.303 * R_G[:,2] # wrist position

    	    # Calculate joint angles using Geometric IK method
    	    #Calculate angles of revolute joints needed to reach desired Position
            theta1 = atan2(W_pos[1], W_pos[0]) #arc tan of wrist y,x position
            theta1_debug += (' ' + str(theta1) + '|')

            #Needed vars to solve for theta 2 & 3, using triangle and trig identities
            side1 = 1.5   #distance between Wrist and joint 2 when frame at origin
            #distance between Wrist and Joint 2
            side2 = sqrt(pow((sqrt(pow(W_pos[0], 2) + pow(W_pos[1], 2))- 0.35) ,2) + pow((W_pos[2] - 0.75), 2))
            side3 = 1.25  #distance between joint 2 and 3
            #Triangle created using the 3 sides, use law of cosines to evaluate
            angle1 = acos((pow(side2, 2) + pow(side3, 2) - pow(side1, 2)) / (2*side3*side2)) #angle between side2 and side3
            angle2 = acos((pow(side1, 2) + pow(side3, 2) - pow(side2, 2)) / (2*side3*side1)) #angle between side1 and side3
            angle3 = pi - angle1 - angle2 #angle between side1 and side2 knowing sum of angles of triangle = pi  (180 degrees)

            #Calculate theta2 & theta3 using angles and wrist position
            theta2 = (pi/2) - angle1 - atan2(W_pos[2] - 0.75, (sqrt(pow(W_pos[0], 2) + pow(W_pos[1], 2))- 0.35))
            theta2_debug += (' ' + str(theta2) + '|')
            theta3 = (pi/2) - (angle2+0.036)
            # if theta4 > (2*pi):
            #     theta4 = theta4 - 2*pi
            # if theta4 < (-2*pi):
            #     theta4 = theta4 + 2*pi
            theta3_debug += (' ' + str(theta3) + '|')

            #Use Rotation Matixes and Euler Angles to solve theata 4,5, & 6
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = R0_3.transpose() *  R_G  #take inverse of R0_3 and multiply by Rotation of gripper

            #theta4 = atan2(R3_6[2,2], -R3_6[0,2])

            # if theta4 > (2*pi):
            #     theta4 = theta4 - 2*pi
            # if theta4 < (-2*pi):
            #     theta4 = theta4 + 2*pi
            #theta4_debug += (' ' + str(theta4) + '|')

            theta5 = atan2(sqrt(pow(R3_6[0,2], 2) + pow(R3_6[2,2], 2)), R3_6[1,2])
            #Multiple joint solutions for theta 4 and theta 6 since joints share same joint z-axis
            if theta5 < 0:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            else:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])

            # if theta5 > (2*pi):
            #     theta5 = theta5 - 2*pi
            # if theta5 < (-2*pi):
            #     theta5 = theta5 + 2*pi
            theta4_debug += (' ' + str(theta4) + '|')
            theta5_debug += (' ' + str(theta5) + '|')

            #theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            # if theta6 > (2*pi):
            #     theta6 = theta6 - 2*pi
            # if theta6 < (-2*pi):
            #     theta6 = theta6 + 2*pi
            theta6_debug += (' ' + str(theta6) + '|')


	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)


        # print(theta4_debug)
        # print(theta5_debug)
        # print(theta6_debug)
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
