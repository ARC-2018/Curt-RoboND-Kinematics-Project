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
    # s.update({q3: 0, q4: 0, q5: 0, q6: 0})
    # s.update({q4: 0, q5: 0, q6: 0})

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

    global T0_2, T0_3
    T0_2 = (T0_1 * T1_2)
    T0_3 = (T0_2 * T2_3)
    T0_4 = (T0_3 * T3_4)
    T0_5 = (T0_4 * T4_5)
    T0_6 = (T0_5 * T5_6)
    T0_G = (T0_6 * T6_G)

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

            Rq = tf.transformations.quaternion_matrix(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            theta1, theta2, theta3, theta4, theta5, theta6 = do_work(px, py, pz, roll, pitch, yaw, Rq)
            print "thetas are", theta1, theta2, theta3, theta4, theta5, theta6

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
    if 1:
        do_test()
    print "Ready to receive an IK request"
    rospy.spin()

def do_test():
    px,py,pz = 2.0900910021952077, 0.900061404046043, 2.345040102031809
    roll, pitch, yaw = -0.0006983567709816882, 0.0006055558123970211, -0.0008013688952962057
    # roll, pitch, yaw = -0.2, 0.3, 0.4
    px,py,pz = 2.0, 2.0, 2.0
    roll, pitch, yaw = 0.0, 0.0, 0.0
    theta1, theta2, theta3, theta4, theta5, theta6 = do_work(px, py, pz, roll, pitch, yaw, None)
    print "thetas are", theta1, theta2, theta3, theta4, theta5, theta6
    sys.exit(0)

def do_work(px, py, pz, roll, pitch, yaw, Rq):

    print "start do work"
    print "px,py,pz end is", px, py, pz
    print "roll, pitch, yaw is", roll, pitch, yaw

    #################################
    # Back calculate wrist position
    #################################

    global Wrist_length

    # R_rpy = rot_x(roll)*rot_y(pitch)*rot_z(yaw) # Not correct -- intrinsic
    R_ypr = rot_z(yaw)*rot_y(pitch)*rot_x(roll) # Correct for rpy we are given

    # R_sxyz = tf.transformations.euler_matrix(roll, pitch, yaw, "sxyz")
    # R_rxyz = tf.transformations.euler_matrix(roll, pitch, yaw, "rxyz")
    # R_szyx = tf.transformations.euler_matrix(yaw, pitch, roll, "szyx")
    # R_rzyx = tf.transformations.euler_matrix(yaw, pitch, roll, "rzyx")

    # print "Rq", Rq
    # print "R_rpy", R_rpy
    # print "R_ypr", R_ypr
    # print "R_sxyz", R_sxyz
    # print "R_rxyz", R_rxyz
    # print "R_szyx", R_szyx
    # print "R_rzyx", R_rzyx

    if 0:
        for t in "rxyz rzyx sxyz szyx".split():
            print " Transform", t
            r,p,y = tf.transformations.euler_from_matrix(np.array(R_ypr).astype(np.float64), axes=t)
            print t, "on R_ypr are", r, p, y

    R0_6 = R_ypr
    print "R0_6 done correctly", R0_6

    T0_6 = R0_6.row_join(Matrix([px, py, pz])).col_join(Matrix([[0,0,0,1]]))

    # print "T0_6", T0_6

    #, calculate wrist ceinter as -Wrist_length along the z axis
    # wc = T0_6 * Matrix([0.0, 0.0, -Wrist_length, 1.0])
    #, calculate wrist ceinter as -Wrist_length along the x axis
    wc = T0_6 * Matrix([-Wrist_length, 0.0, 0.0, 1.0])

    print "px,py,pz is", px, py, pz
    print "Wrist center is", wc

    #################################
    # Calculate a few joint angles
    #################################

    theta1 = 0.0
    theta2 = 0.0
    theta3 = 0.0
    theta4 = 0.0
    theta5 = 0.0
    theta6 = 0.0
    theta7 = 0.0

    theta1 = atan2(wc[1], wc[0]).evalf()
    to_deg = 180.0 / pi
    to_rad = pi / 180.0

    print "theta1 is", (theta1*to_deg).evalf(), "degres"

    global q1, q2, q3, q4, q5, q6, q7 
    global T0_2, T0_3

    t2 = T0_2.evalf(subs={q1: theta1, q2: 0})
    o2 = simplify(t2*Matrix([0,0,0,1])).evalf()
    print "o2 is", o2

    l34 = sqrt(.054**2 + 1.5**2) # Straignt line length from O3 to O4
    l23 = 1.25
    l24 = sqrt((wc[0]-o2[0])**2 + (wc[1]-o2[1])**2 + (wc[2]-o2[2])**2)

    print "tri sides (l34, l23, l24)", l34, l23, l24

    # Law of cosines to calculate triangle angle from sides

    o3a = acos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
    o2a = acos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
    o4a = simplify(pi - o3a - o2a).evalf()

    print "triangles are", (o3a*to_deg).evalf(), (o2a*to_deg).evalf(), (o4a*to_deg).evalf()

    # o2a2 = atan2(wc[2]-o2[2], wc[0]-o2[0])
    o2a2 = atan2(wc[2]-o2[2], sqrt((wc[0]-o2[0])**2 + (wc[1]-o2[1])**2))

    theta2 = (pi/2 - (o2a2 + o2a)).evalf() # straight up is zero for robot
    theta2 = theta2.evalf()

    print "Theta2 is", (theta2*to_deg).evalf(), " which is sum of o2a and o2a2", (o2a * to_deg).evalf(), (o2a2 * to_deg).evalf()

    t2 = T0_2.evalf(subs={q1: theta1, q2: theta2-pi/2})
    print "q2 is", ((theta2-pi/2) * to_deg).evalf()
    o3 = t2*Matrix([1.25, 0.0, 0.0, 1])
    print "o3 is", o3

    # We know know the world coordinates for the location of o3 and the wrist center wc (o4)
    # Angle from o3 to o4 due to arm offset of .054 when 
    o3o4 = atan2(0.054, 1.5).evalf()

    theta3 = (pi/2 - (o3a + o3o4)).evalf()
    
    print "o3o4 offset angle is", (o3o4*to_deg).evalf(), "theta3 is", (theta3*to_deg).evalf()

    t3 = T0_3.evalf(subs={q1: theta1, q2: theta2-pi/2, q3: theta3})
    o4 = simplify(t3*Matrix([-0.054, 1.5, 0.0, 1])).evalf()

    print "o4 is", o4
    print "wc (should be same as 04)", wc

    # lets do some triple checking now that we calucated 04 from thje anlges but found it to be about
    # 1 inch off from the wc we were trying to calucate4 angles for.  Is there an errror in the
    # code or is there really this much numercial error in the calcuations????

    theta1o2 = atan2(o2[1], o2[0])
    theta1o3 = atan2(o3[1], o3[0])
    theta1o4 = atan2(o4[1], o4[0])
    theta1wc = atan2(wc[1], wc[0])

    print "theta1 is", theta1, "and theta1o4,o2,wc,o3", theta1o4, theta1o2, theta1wc, theta1o3

    # What's the distance from o3 to o4?  and o3 to wc?
    o3o4distance = sqrt((o3[0]-o4[0])**2 + (o3[1]-o4[1])**2 + (o3[2]-o4[2])**2)
    print "distance from o3 to o4 is", o3o4distance, "should be same as l34:", l34, "which should be a little bit larget than 1.5"
    o3wcdistance = sqrt((o3[0]-wc[0])**2 + (o3[1]-wc[1])**2 + (o3[2]-wc[2])**2)
    print "distance from o3 to wc is", o3wcdistance, "should be same as l34:", l34, "which should be a little bit larget than 1.5"

    print "distance from o2 to o3", distance(o2, o3)
    print "distance from o2 to wc", distance(o2, wc)
    print "distance from o2 to o4", distance(o2, o4)
    print "distance from 000 to o2", distance(Matrix([0,0,0]), o2)
    print "distance from .35/.75", sqrt(.35**2 + .75**2)

    if 0:
        # OK, calculate the triangle angle and sides again using o2, o3, and o4
        l23 = distance(o2, o3)
        l34 = distance(o3, o4)
        l24 = distance(o2, o4)
        print "NEW tri sides (l34, l23, l24)", l34, l23, l24
        o3a = acos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
        o2a = acos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
        o4a = simplify(pi - o3a - o2a).evalf()
        print "NEW triangles are", (o3a*to_deg).evalf(), (o2a*to_deg).evalf(), (o4a*to_deg).evalf()

    #############################################
    # Now q4 q5 and q6 for the end effector
    #############################################

    # T3_6 = T0_3.transpose()[:3,:3] * R_ypr
    R3_6 = t3.transpose()[:3,:3] * R_ypr

    print "R3_6 is", R3_6
    print "R3_6*000 WL is", R3_6 * Matrix([0.0, 0.0, Wrist_length])

    a, b, g = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), axes='rxyx')
    # a, b, g = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), axes='sxyz')

    print "abg from R3_6 is", a, b, g
    print "abg from R3_6 is", r_to_d(a), r_to_d(b), r_to_d(g)

    theta4 = a
    theta5 = b
    theta6 = g

    # theta4 = d_to_r(100)
    # theta5 = d_to_r(-50)
    # theta6 = 0
    
    tt = T_total.evalf(subs={q1: theta1, q2: theta2-pi/2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    tt_end = tt * Matrix([0,0,0,1])

    print "tt is", tt
    print "tt_end is", tt_end
    print "distance from tt_end to wc is", distance(wc, tt_end)
    print "distance from tt_end to pxyz is", distance(tt_end, Matrix([px, py, pz]))
    
    return theta1, theta2, theta3, theta4, theta5, theta6

def distance(p1, p2):
    return (sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)).evalf()

def r_to_d(x):
    return (x * 180.0 / pi).evalf()

def d_to_r(x):
    return (x * pi / 180.0).evalf()


if __name__ == "__main__":
    IK_server()
