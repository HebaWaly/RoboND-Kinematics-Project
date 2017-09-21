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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
    
        ### FK ###
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')   
        # Create Modified DH parameters
        s = {alpha0:      0, a0:      0, d1:  0.75, q1:          q1,
             alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2. + q2,
             alpha2:      0, a2:   1.25, d3:     0, q3:          q3,
             alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:          q4,
             alpha4:  pi/2., a4:      0, d5:     0, q5:          q5,
             alpha5: -pi/2., a5:      0, d6:     0, q6:          q6,
             alpha6:      0, a6:      0, d7: 0.303, q7:           0}            
        # Define Modified DH Transformation matrix
        def transform(alpha, a, d, q):
            T = Matrix([[           cos(q),           -sin(q),          0,              a],
                  [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                  [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                  [                0,                 0,           0,             1]])
            return T
        # Create individual transformation matrices
        T0_1 = transform(alpha0, a0, d1, q1).subs(s)
        T1_2 = transform(alpha1, a1, d2, q2).subs(s)
        T2_3 = transform(alpha2, a2, d3, q3).subs(s)
        T3_4 = transform(alpha3, a3, d4, q4).subs(s)
        T4_5 = transform(alpha4, a4, d5, q5).subs(s)
        T5_6 = transform(alpha5, a5, d6, q6).subs(s)
        T6_G = transform(alpha6, a6, d7, q7).subs(s)

        T0_2 = T0_1 * T1_2
        T0_3 = T0_2 * T2_3
        T0_4 = T0_3 * T3_4
        T0_5 = T0_4 * T4_5
        T0_6 = T0_5 * T5_6
        T0_G = T0_6 * T6_G
        ### FK end ###

        ### IK ###
        R_x = Matrix([[ 1,              0,        0],
                      [ 0,        cos(q1), -sin(q1)],
                      [ 0,        sin(q1),  cos(q1)]])

        R_y = Matrix([[ cos(q2),        0,  sin(q2)],
                      [       0,        1,        0],
                      [-sin(q2),        0,  cos(q2)]])

        R_z = Matrix([[ cos(q3), -sin(q3),        0],
                      [ sin(q3),  cos(q3),        0],
                      [ 0,              0,        1]])
        R_correction = R_z.subs(q3, pi) * R_y.subs(q2, -pi/2.)
        Rot_G = R_z * R_y * R_x
        Rot_G = Rot_G * R_correction

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
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
            # Rotation Angles:
            Rot_G = Rot_G.subs({q3: yaw, q2: pitch, q1: roll})
            
            # Calculate joint angles using Geometric IK method
            EE = Matrix([[px],
                         [py],
                         [pz]])
            WC = EE - 0.303 * Rot_G[:,2]
            
            theta1 = atan2(WC[1],WC[0])
            
            s1 = sqrt(pow(WC[0],2) + pow(WC[1],2)) - 0.35 #WC[0] - 0.35
            s2 = WC[2] - 0.75
            s3 = sqrt(pow(s1, 2) + pow(s2, 2))
            s4 = sqrt(pow(-0.054, 2) + pow(1.5, 2))
            s5 = 1.25
            
            beta1 = atan2(s2, s1)
            beta2 = acos((pow(s3, 2) + pow(s5, 2) - pow(s4, 2))/(2*s3*s5))
            theta2 = pi/2. - beta1 - beta2
          
            beta3 = acos((pow(s4, 2) + pow(s5, 2) - pow(s3, 2))/(2*s4*s5))
            beta4 = atan2(-0.054,1.5)
            theta3 = pi/2. - beta3 - beta4
            
            R0_3 = T0_3[0:3,0:3].evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv("LU") * Rot_G
            
            # Also R3_6 can be calculated from: T3_6 = T3_4*T4_5*T5_6
            # solving both sides:
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(pow(R3_6[0,2],2)+pow(R3_6[2,2],2)),R3_6[1,2])
            ###

  
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
