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

def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                [0, cos(q), -sin(q)],
                [0, sin(q), cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[cos(q), 0, sin(q)],
                [0, 1, 0],
                [-sin(q), 0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[cos(q), -sin(q), 0],
                [sin(q), cos(q), 0],
                [0, 0, 1]])
    
    return R_z

DH_s = None
T_Total = None
Wrist_length = 0

def do_forward():
    global q1, q2, q3, q4, q5, q6, q7 
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            
    s = {alpha0: 0,      a0: 0,
        alpha1: -pi/2,  a1: 0.35,   d1: 0.75,
        alpha2: 0,      a2: 1.25,   d2: 0,
        alpha3: -pi/2,  a3: -0.054, d3: 0,
        alpha4: pi/2,   a4: 0,      d4: 1.5,
        alpha5: -pi/2,  a5: 0,      d5: 0,
        alpha6: 0,      a6: 0,      d6: 0,
                                    d7: 0.303,  q7: 0}
    global DH_s
    DH_s = s
    # s.update({q1: 0, q2: -pi/2, q3: 0, q4: 0, q5: 0, q6: 0})
    # s.update({q2: -pi/2, q3: 0, q4: 0, q5: 0, q6: 0})
    s.update({q3: 0, q4: 0, q5: 0, q6: 0})

    T0_1 = Matrix([ [cos(q1),              -sin(q1),            0,              a0],
                    [sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0), -sin(alpha0),   -sin(alpha0)*d1],
                    [sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0), cos(alpha0),   cos(alpha0)*d1],
                    [0,                     0,                  0,              1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([ [cos(q2),              -sin(q2),            0,              a1],
                    [sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1), -sin(alpha1),   -sin(alpha1)*d2],
                    [sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1), cos(alpha1),   cos(alpha1)*d2],
                    [0,                     0,                  0,              1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([ [cos(q3),              -sin(q3),            0,              a2],
                    [sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2), -sin(alpha2),   -sin(alpha2)*d3],
                    [sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2), cos(alpha2),   cos(alpha2)*d3],
                    [0,                     0,                  0,              1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([ [cos(q4),              -sin(q4),            0,              a3],
                    [sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3), -sin(alpha3),   -sin(alpha3)*d4],
                    [sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3), cos(alpha3),   cos(alpha3)*d4],
                    [0,                     0,                  0,              1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([ [cos(q5),              -sin(q5),            0,              a4],
                    [sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4), -sin(alpha4),   -sin(alpha4)*d5],
                    [sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4), cos(alpha4),   cos(alpha4)*d5],
                    [0,                     0,                  0,              1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([ [cos(q6),              -sin(q6),            0,              a5],
                    [sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5), -sin(alpha5),   -sin(alpha5)*d6],
                    [sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5), cos(alpha5),   cos(alpha5)*d6],
                    [0,                     0,                  0,              1]])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([ [cos(q7),              -sin(q7),            0,              a6],
                    [sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6), -sin(alpha6),   -sin(alpha6)*d7],
                    [sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6), cos(alpha6),   cos(alpha6)*d7],
                    [0,                     0,                  0,              1]])
    T6_G = T6_G.subs(s)

    global T0_2
    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_G = simplify(T0_6 * T6_G)

    # Rotations to transform(/fix) final DH axes of gripper to world axes
    # Rotate about z by 180 deg
    R_z = Matrix([  [cos(pi),    -sin(pi),    0,      0],
                    [sin(pi),    cos(pi),     0,      0],
                    [0,             0,              1,      0],
                    [0,             0,              0,      1]])

    # Rotate about y by -90 deg
    R_y = Matrix([  [cos(-pi/2),     0,      sin(-pi/2),  0],
                    [0,                 1,      0,              0],
                    [-sin(-pi/2),    0,      cos(-pi/2),  0],
                    [0,                 0,      0,              1]])

    R_corr = simplify(R_z * R_y)

    global T_total
    T_total = simplify(T0_G * R_corr)

    global Wrist_length
    Wrist_length = s[d7]


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
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
     
            # Calculate joint angles using Geometric IK method
            theta1, theta2, theta3, theta4, theta5, theta6 = do_work(px, py, pz, roll, pitch, yaw)

            # Populate response for the IK request

	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Calculate Forward Kinematic Matrix"
    do_forward()
    do_test()
    print "Ready to receive an IK request"
    rospy.spin()

def do_test():
    px,py,pz = 2.0900910021952077, 0.900061404046043, 2.345040102031809
    roll, pitch, yaw = -0.0006983567709816882, 0.0006055558123970211, -0.0008013688952962057
    theta1, theta2, theta3, theta4, theta5, theta6 = do_work(px, py, pz, roll, pitch, yaw)
    print "thetas are", theta1, theta2, theta3, theta4, theta5, theta6
    sys.exit(0)

def do_work(px, py, pz, roll, pitch, yaw):

    print "start do work"
    print "px,py,pz end is", px, py, pz
    print "roll, pitch, yaw is", roll, pitch, yaw

    # Back caclculate wrist position

    global Wrist_length

    print "rotx(roll)", rot_x(roll)

    ee_rot = simplify(rot_x(roll)*rot_y(pitch)*rot_z(yaw))

    print "total rpy rotate", ee_rot

    r_complete = ee_rot.row_join(Matrix([px, py, pz])).col_join(Matrix([[0,0,0,1]]))

    print "r_complete is", r_complete

    #, calculate wrist ceinter as -Wrist_length along the z axis
    wc = simplify(r_complete * Matrix([0.0, 0.0, -Wrist_length, 1]))

    print "Wrist center is", wc

    # Calculate a few joint angles
    theta1 = 0.0
    theta2 = 0.0
    theta3 = 0.0
    theta4 = 0.0
    theta5 = 0.0
    theta6 = 0.0
    theta7 = 0.0

    theta1 = atan2(wc[1], wc[0])
    to_deg = 180.0 / 3.1415926
    to_rad = 3.1415926 / 180.0

    print "theta1 is", simplify(theta1*to_deg), "degres"

    theta2 = -80 * to_rad
    theta3 = 0 * to_rad

    global q1, q2, q3, q4, q5, q6, q7 
    global T0_2

    t2 = T0_2.evalf(subs={q1: theta1, q2: 0})
    o2 = simplify(t2*Matrix([0,0,0,1]))
    print "o2 is", o2

    l34 = sqrt(.054**2 + 1.5**2) # Straignt line length from O3 to O4
    l23 = 1.25
    l24 = sqrt((wc[0]-o2[0])**2 + (wc[1]-o2[1])**2 + (wc[2]-o2[2])**2)

    print "tri sides", l34, l23, l24

    # Law of cosines to calculate triangle angle from sides

    o3a = acos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
    o2a = acos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
    o4a = simplify(pi - o3a - o2a).evalf()

    print "triangles are", o3a*to_deg, o2a*to_deg, o4a*to_deg

    o2a2 = atan2(wc[1]-o2[1], wc[0]-o2[0])

    theta2 = pi/2 - (o2a2 + o2a) # straight up is zero for robot

    print "Theta2 is", (theta2*to_deg).evalf(), " which is sum of o2a and o2a2", o2a * to_deg, o2a2 * to_deg

    t2 = T0_2.evalf(subs={q1: theta1, q2: theta2-pi/2})
    print "q2 is", ((theta2-pi/2) * to_deg).evalf()
    o3 = t2*Matrix([0.0, 0.0, 0.0, 1])
    print "o3 origin is ", o3
    o3 = t2*Matrix([1.25, 0.0, 0.0, 1])
    print "o3 is", o3
    o3 = t2*Matrix([1.00, 0.0, 0.0, 1])
    print "o3 with 1.00 vs 1.25 is", o3
    
    return theta1, theta2, theta3, theta4, theta5, theta6


if __name__ == "__main__":
    IK_server()
