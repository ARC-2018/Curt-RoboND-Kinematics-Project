#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Student: Curt Welch <curt@kcwc.com>
# July 6, 2017

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
import time
import matplotlib.pyplot as plt


class Kuka_KR210:
    def __init__(self):
        self.wrist_length = 0.303
        self.error_data = None
        if 1:
            self.test1()
            self.test1b()
        if 0:
            self.test2()
        if 0:
            self.test3()
        if 0:
            self.test4()
        if 0:
            self.print_report_information() # For the project report

    def joint1_in_range(self, radians):
        deg = self.r_to_d(radians)
        return deg >= -185 and deg <= 185

    def joint2_in_range(self, radians):
        deg = self.r_to_d(radians)
        return deg >= -45 and deg <= 85

    def joint3_in_range(self, radians):
        deg = self.r_to_d(radians)
        return deg >= -210 and deg <= (155-90)

    def joint4_in_range(self, radians):
        deg = self.r_to_d(radians)
        return deg >= -350 and deg <= 350
        
    def joint5_in_range(self, radians):
        deg = self.r_to_d(radians)
        return deg >= -125 and deg <= 125

    def joint6_in_range(self, radians):
        deg = self.r_to_d(radians)
        return deg >= -350 and deg <= 350

    def start_error_plot(self):
        if self.error_data is None:
            self.error_data = [] # indicates we should collect error values

    def show_error_plot(self):

        if self.error_data is not None:
            plt.plot(self.error_data, 'r^')
            plt.ylabel('Position Error')
            plt.show()

        self.error_data = None


    def print_report_information(self):
        last_t = None

        for link in range(1,9):
            t = self.getT_sympy(link, DHT=True)
            print
            print "T%d_%d" % (link-1, link)
            pprint(t)

        px, py, pz, roll, pitch, yaw = symbols('px py pz roll pitch yaw')

        Rrpy = simplify(self.rot_z_sympy(yaw) * self.rot_y_sympy(pitch) * self.rot_x_sympy(roll))

        T = Rrpy.row_join(Matrix([px, py, pz])).col_join(Matrix([0, 0, 0, 1]).T)

        print
        print "Total Transform"
        pprint(T)
        print

        print "## px,   py,    pz  is 2.0900910022 0.900061404046 2.34504010203"
        print "## roll, pitch, yaw is -0.000698356770982 0.000605555812397 -0.000801368895296"
        print "## T_ypr is"
        print "## [[  9.99999496e-01   8.00945720e-04   6.06115075e-04   2.09009100e+00]"
        print "## [ -8.01368663e-04   9.99999435e-01   6.97871217e-04   9.00061404e-01]"
        print "## [ -6.05555775e-04  -6.98356586e-04   9.99999573e-01   2.34504010e+00]"
        print "## [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]"
        print "## Wrist center:"
        print "## [[ 1.78709116]"
        print "## [ 0.90030422]"
        print "## [ 2.34522359]"
        print "## [ 1.        ]]"
        print "## joint angles:  0.4667  0.1558 -0.4343 -1.0712  0.5391  1.0039"
        print "## angles are 0.466668568222 0.155822729589 -0.434265795574 -1.07118579525 0.539118653565 1.00392815792"
        
        s = {}
        s.update({px: 2.0900910022, py: 0.900061404046, pz: 2.34504010203})
        s.update({roll: -0.000698356770982, pitch: 0.000605555812397, yaw: -0.000801368895296})

        tt = T.subs(s).evalf()

        print "tt is"
        pprint(tt)

        wc = tt * Matrix([-0.303, 0, 0, 1])
        print "wc is", wc

        ## Verity T0 to T9
        ## Printout we used has only 4 digits of accuracy for these

        tt = self.getT_sympy(8,
            t1=0.466668568222, t2=0.155822729589, t3=-0.434265795574, t4=-1.07118579525, t5=0.539118653565, t6=1.00392815792)

        print "total from q's is"
        pprint(tt)

        wc = tt * Matrix([-0.303, 0, 0, 1])
        print "wc is", wc
        
        sys.exit(0)

    def test1(self):
        # Test IK code
        if 1:
            # One real simple from runing the demo
            px,py,pz = 2.0900910021952077, 0.900061404046043, 2.345040102031809
            roll, pitch, yaw = -0.0006983567709816882, 0.0006055558123970211, -0.0008013688952962057
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6
        if 1:
            ## DH Home position -- angles all zero
            px,py,pz = 2.153, 0.0, 1.946
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6
        if 1:
            # test for guy on slack that broke my code
            # high test wc inside j2 circle
            # print np.arctan2(0.229042, 0.165865)
            px,py,pz = 0.165865+0.303, 0.229042, 2.5848
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6
        if 1:
            # Low test fixing above high test borke this?
            px,py,pz = 1.0+0.303, 1.0, 0.2
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6

        #sys.exit(0)

    def test1b(self):
        # Test IK code
        if 1:
            # One real simple from runing the demo
            px,py,pz = 2.0900910021952077, 0.900061404046043, 2.345040102031809
            roll, pitch, yaw = -0.0006983567709816882, 0.0006055558123970211, -0.0008013688952962057
            theta1, theta2, theta3, theta4, theta5, theta6 = test_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6
        if 1:
            ## DH Home position -- angles all zero
            px,py,pz = 2.153, 0.0, 1.946
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = test_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6
        if 1:
            # test for guy on slack that broke my code
            # high test wc inside j2 circle
            # print np.arctan2(0.229042, 0.165865)
            px,py,pz = 0.165865+0.303, 0.229042, 2.5848
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = test_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6
        if 1:
            # Low test fixing above high test borke this?
            px,py,pz = 1.0+0.303, 1.0, 0.2
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = test_IK(px, py, pz, roll, pitch, yaw, debug=True)
            print "angles are", theta1, theta2, theta3, theta4, theta5, theta6

        sys.exit(0)

    def test_angles(self, angle_list):

        last_thetas = None

        for a in angle_list:
            print "----------------------------------------------------------------------"
            print "In test1, starting angles are", a
            px, py, pz, roll, pitch, yaw = self.do_FK(a)
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, last_thetas=last_thetas, debug=True)
            last_thetas = (theta1, theta2, theta3, theta4, theta5, theta6)

    def test2(self):

        # This set of numbers failed to produce the correct wrist roations
        # so I've captured the log output and turned it into a test case.
        
        angle_list = []

        angle_list.append([0.2174, 0.0370, -0.1521, 2.6957, 0.2989, -2.7292])
        angle_list.append([0.2448, 0.0503, -0.1755, 2.9590, 0.3327, -2.9958])

        # Bug was here, next should have joint 6 of -3 -- found bug and fixed it...

        angle_list.append([0.2629, 0.0589, -0.1912, 3.1268, 0.3536, 3.1174])
        angle_list.append([0.2814, 0.0674, -0.2076, 3.2944, 0.3736, 2.9478])

        self.test_angles(angle_list)

        sys.exit(0)

    def test3(self):
        # An over the head reach test

        print "test3() -- an over the reach test that causes a base flip"

        angle_list = []
        # angle_list.append([0.0001, -0.0882, 0.0850, 0.0000, 0.0033, 0.0001])
        # angle_list.append([0.1320, -0.1152, -0.0175, 0.0748, -0.0114, 0.0572])
        # angle_list.append([0.2641, -0.1431, -0.1191, 0.1314, -0.0260, 0.1325])
        # angle_list.append([0.3963, -0.1721, -0.2194, 0.1886, -0.0404, 0.2071])
        # angle_list.append([0.5286, -0.2022, -0.3184, 0.2444, -0.0546, 0.2830])
        # angle_list.append([0.6612, -0.2335, -0.4156, 0.2976, -0.0685, 0.3614])
        # angle_list.append([0.7941, -0.2663, -0.5106, 0.3469, -0.0824, 0.4434])
        # angle_list.append([0.9274, -0.3009, -0.6028, 0.3910, -0.0961, 0.5303])
        # angle_list.append([1.0615, -0.3378, -0.6911, 0.4279, -0.1100, 0.6236])
        # angle_list.append([1.1799, -0.3723, -0.7642, 0.4523, -0.1225, 0.7114])
        # angle_list.append([1.3017, -0.4097, -0.8322, 0.4674, -0.1357, 0.8052])
        
        angle_list.append([1.4368, -0.4506, -0.8939, 0.4684, -0.1508, 0.8999])
        angle_list.append([-3.0150, -0.4942, -0.9446, -0.1602, -0.2005, -0.0786]) # Base flip 
        angle_list.append([-1.4371, -0.4340, -0.8050, -0.6709, -0.1373, -0.7225])
        angle_list.append([-1.2128, -0.3768, -0.6506, -0.6642, -0.1464, -0.5273])

        # angle_list.append([-1.0297, -0.3290, -0.5158, -0.6109, -0.1570, -0.4049])
        # angle_list.append([-0.8480, -0.2799, -0.3790, -0.5291, -0.1693, -0.3096])
        # angle_list.append([-0.6838, -0.2335, -0.2547, -0.4387, -0.1816, -0.2388])
        # angle_list.append([-0.5199, -0.1845, -0.1310, -0.3386, -0.1949, -0.1773])
        # angle_list.append([-0.3900, -0.1433, -0.0338, -0.2553, -0.2064, -0.1321])
        # angle_list.append([-0.2601, -0.0997, 0.0624, -0.1705, -0.2187, -0.0879])
        # angle_list.append([-0.1301, -0.0533, 0.1572, -0.0861, -0.2320, -0.0432])
        # angle_list.append([-0.0002, -0.0035, 0.2503, -0.0033, -0.2463, 0.0038])

        self.test_angles(angle_list)

        sys.exit(0)

    def test4(self):
        # A far reach back over head that caused the code to crap out
        # Turned out to be a point too far to reach -- added code to catch it and make
        # the arm reach as far as possible in the intented direction.

        ## [INFO] [1499400762.939037, 1210.739000]: IK joint angles:  0.6136 -0.2437 -1.3023 -0.2707 -1.7020 -3.3512
        ## [INFO] [1499400762.946252, 1210.741000]: IK joint angles:  1.2025 -0.0245 -1.4983 -0.5815 -1.6759 -3.0743
        ## ./IK_server.py:657: RuntimeWarning: invalid value encountered in arccos
        ## o3a = np.arccos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
        ## ./IK_server.py:658: RuntimeWarning: invalid value encountered in arccos
        ## o2a = np.arccos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
        ## ERROR -- final gripper location fails to match target
        ## target px, py, pz 0.0135223310093 0.626326794872 3.48501445893
        ## result px, py, pz nan nan nan
        ## ERROR -- final gripper pose fails to match target
        ## target roll pitch yaw: 0.424568603719 -0.106001542291 -2.15875618369
        ## result roll pitch yaw: nan nan 0.0
        ## [INFO] [1499400762.957580, 1210.744000]: IK joint angles:  1.3677     nan     nan  0.0000     nan     nan
        ## [INFO] [1499400762.963179, 1210.746000]: IK joint angles:  1.4576  0.2836 -1.4635 -3.2539  1.6309 -5.7097
        ## [INFO] [1499400762.968141, 1210.748000]: IK joint angles:  1.5044  0.3112 -1.3243 -3.0662  1.6589 -5.5798
        ## [INFO] [1499400762.980311, 1210.750000]: IK joint angles:  1.5469  0.3470 -1.2031 -2.8724  1.6784 -5.4488

        angle_list = []
        angle_list.append([0.6136, -0.2437, -1.3023, -0.2707, -1.7020, -3.3512]) # good
        angle_list.append([1.2025, -0.0245, -1.4983, -0.5815, -1.6759, -3.0743]) # good
        self.test_angles(angle_list)

        print
        print "DIE DIE DIE from too far to reach ============================="
        print

        px = 0.0135223310093
        py = 0.626326794872
        pz = 3.48501445893
        roll = 0.424568603719
        pitch = -0.106001542291
        yaw = -2.15875618369

        theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, last_thetas=None, debug=True)

        print
        print "==============================================================="
        print

        angle_list = []
        angle_list.append([1.4576, 0.2836, -1.4635, -3.2539, 1.6309, -5.7097]) # good
        angle_list.append([1.5044, 0.3112, -1.3243, -3.0662, 1.6589, -5.5798]) # good
        self.test_angles(angle_list)

        sys.exit(0)


    #
    # Create rotation matrix for each link
    #
    def getR(self, n, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None):
        # Get rotation matrix for different arm frames, n=1 to 8
        r = self.getT(n, t1=t1, t2=t2, t3=t3, t4=t4, t5=t5, t6=t6)
        return r[:3,:3]

    #
    # Create Homogeneous transformation matrix
    # Re-coded to use only numpy instead of sympy
    #
    def getT(self, n, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None):

        # Return Homogeneous transformation matrix for arm frame.
        # n -- controls which frame matrix is returned (1 to 8)
        # n=1 returns T0_1, n=2 returns T0_2, ... n=7 returns T0_G, n=8 returns T0_world
        # T0_G has griper hand at 0,0 with x up, z forward
        # T0_world is same as T0_G except axis rotated to world frame of z up, x forward

        if n == 0:
            return np.identity(4)

        # print "Calculate Forward Kinematic Matrix for n=", n

        # DH table parameters
                
        alpha0 = 0
        a0 = 0

        alpha1 = -np.pi/2
        a1 = 0.35
        d1 = 0.75
        q1 = t1

        alpha2 = 0
        a2 = 1.25
        d2 = 0
        q2 = t2-np.pi/2

        alpha3 = -np.pi/2
        a3 = -0.054
        d3 = 0
        q3 = t3

        alpha4 = np.pi/2
        a4 = 0
        d4 = 1.5
        q4 = t4

        alpha5 = -np.pi/2
        a5 = 0
        d5 = 0
        q5 = t5

        alpha6 = 0
        a6 = 0
        d6 = 0
        q6 = t6

        d7 = 0.303
        q7 = 0

        T0_1 = self.getDHT(alpha0, a0, d1, q1)
        if n == 1:
            return T0_1

        T0_2 = T0_1 * self.getDHT(alpha1, a1, d2, q2)
        if n == 2:
            return T0_2

        T0_3 = T0_2 * self.getDHT(alpha2, a2, d3, q3)
        if n == 3:
            return T0_3

        T0_4 = T0_3 * self.getDHT(alpha3, a3, d4, q4)
        if n == 4:
            return T0_4

        T0_5 = T0_4 * self.getDHT(alpha4, a4, d5, q5)
        if n == 5:
            return T0_5

        T0_6 = T0_5 * self.getDHT(alpha5, a5, d6, q6)
        if n == 6:
            return T0_6

        T0_G = T0_6 * self.getDHT(alpha6, a6, d7, q7)
        if n == 7:
            return T0_G

        # Rotations to transform(/fix) final DH axes of gripper to world axes
        # Rotate about z by 180 deg
        R_z = np.matrix([  [np.cos(np.pi),    -np.sin(np.pi),    0,      0],
                        [np.sin(np.pi),    np.cos(np.pi),     0,      0],
                        [0,             0,              1,      0],
                        [0,             0,              0,      1]])

        # Rotate about y by -90 deg
        R_y = np.matrix([  [np.cos(-np.pi/2),     0,      np.sin(-np.pi/2),  0],
                        [0,                 1,      0,              0],
                        [-np.sin(-np.pi/2),    0,      np.cos(-np.pi/2),  0],
                        [0,                 0,      0,              1]])


        R_corr = (R_z * R_y)

        T_total = (T0_G * R_corr)

        return T_total

    #
    # Build a DH transform matrix from parameters
    #
    def getDHT(self, alpha, a, d, theta):
        return np.matrix([[np.cos(theta),              -np.sin(theta),            0,              a],
                    [np.sin(theta)*np.cos(alpha),  np.cos(theta)*np.cos(alpha), -np.sin(alpha),   -np.sin(alpha)*d],
                    [np.sin(theta)*np.sin(alpha),  np.cos(theta)*np.sin(alpha), np.cos(alpha),   np.cos(alpha)*d],
                    [0,                     0,                  0,              1]])

    #
    # The old version of the getT() based on sympy
    # too slow to use
    #
    def getT_sympy(self, n, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None, DHT=False):

        # Old version that used sympy --- worked fine, just way too slow for my taste (300 times slower than numpy)

        # Return Homogeneous transformation matrix for arm frames
        # n controls which frame matrix is returned (1 to 8)
        # n=1 returns T0_1, n=2 returns T0_2, ... n=7 returns T0_G, n=8 returns T0_world
        # T0_G has griper hand at 0,0 with x up, z forward
        # T0_world is same as T0_G except axis rotated to world frame of z up, x forward

        # if DHT is True return the transform for only the single link

        if n == 0:
            return eye(4)

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # print "Calculate Forward Kinematic Matrix for n=", n
                
        s = {alpha0: 0,      a0: 0,
            alpha1: -pi/2,  a1: 0.35,   d1: 0.75,
            alpha2: 0,      a2: 1.25,   d2: 0,
            alpha3: -pi/2,  a3: -0.054, d3: 0,
            alpha4: pi/2,   a4: 0,      d4: 1.5,
            alpha5: -pi/2,  a5: 0,      d5: 0,
            alpha6: 0,      a6: 0,      d6: 0,
                                        d7: 0.303,  q7: 0}

        # s.update({q1: 0, q2: -pi/2, q3: 0, q4: 0, q5: 0, q6: 0})
        # s.update({q2: -pi/2, q3: 0, q4: 0, q5: 0, q6: 0})
        # s.update({q3: 0, q4: 0, q5: 0, q6: 0})
        # s.update({q4: 0, q5: 0, q6: 0})

        if t1 is not None:
            s.update({q1: t1})
        if t2 is not None:
            s.update({q2: t2-pi/2})
        if t3 is not None:
            s.update({q3: t3})
        if t4 is not None:
            s.update({q4: t4})
        if t5 is not None:
            s.update({q5: t5})
        if t6 is not None:
            s.update({q6: t6})

        T0_1 = Matrix([ [cos(q1),              -sin(q1),            0,              a0],
                        [sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0), -sin(alpha0),   -sin(alpha0)*d1],
                        [sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0), cos(alpha0),   cos(alpha0)*d1],
                        [0,                     0,                  0,              1]])
        T0_1 = T0_1.subs(s).evalf()
        if n == 1:
            if DHT:
                return T0_1
            return T0_1

        T1_2 = Matrix([ [cos(q2),              -sin(q2),            0,              a1],
                        [sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1), -sin(alpha1),   -sin(alpha1)*d2],
                        [sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1), cos(alpha1),   cos(alpha1)*d2],
                        [0,                     0,                  0,              1]])
        T1_2 = T1_2.subs(s).evalf()
        T0_2 = (T0_1 * T1_2).evalf()
        if n == 2:
            if DHT:
                return T1_2
            return T0_2

        T2_3 = Matrix([ [cos(q3),              -sin(q3),            0,              a2],
                        [sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2), -sin(alpha2),   -sin(alpha2)*d3],
                        [sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2), cos(alpha2),   cos(alpha2)*d3],
                        [0,                     0,                  0,              1]])
        T2_3 = T2_3.subs(s).evalf()
        T0_3 = (T0_2 * T2_3).evalf()
        if n == 3:
            if DHT:
                return T2_3
            return T0_3

        T3_4 = Matrix([ [cos(q4),              -sin(q4),            0,              a3],
                        [sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3), -sin(alpha3),   -sin(alpha3)*d4],
                        [sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3), cos(alpha3),   cos(alpha3)*d4],
                        [0,                     0,                  0,              1]])
        T3_4 = T3_4.subs(s).evalf()
        T0_4 = (T0_3 * T3_4).evalf()
        if n == 4:
            if DHT:
                return T3_4
            return T0_4

        T4_5 = Matrix([ [cos(q5),              -sin(q5),            0,              a4],
                        [sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4), -sin(alpha4),   -sin(alpha4)*d5],
                        [sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4), cos(alpha4),   cos(alpha4)*d5],
                        [0,                     0,                  0,              1]])
        T4_5 = T4_5.subs(s)
        T0_5 = (T0_4 * T4_5)
        if n == 5:
            if DHT:
                return T4_5
            return T0_5

        T5_6 = Matrix([ [cos(q6),              -sin(q6),            0,              a5],
                        [sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5), -sin(alpha5),   -sin(alpha5)*d6],
                        [sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5), cos(alpha5),   cos(alpha5)*d6],
                        [0,                     0,                  0,              1]])
        T5_6 = T5_6.subs(s)
        T0_6 = (T0_5 * T5_6)
        if n == 6:
            if DHT:
                return T5_6
            return T0_6

        T6_G = Matrix([ [cos(q7),              -sin(q7),            0,              a6],
                        [sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6), -sin(alpha6),   -sin(alpha6)*d7],
                        [sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6), cos(alpha6),   cos(alpha6)*d7],
                        [0,                     0,                  0,              1]])
        T6_G = T6_G.subs(s)
        T0_G = (T0_6 * T6_G)
        if n == 7:
            if DHT:
                return T6_G
            return T0_G

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


        R_corr = (R_z * R_y)

        T_total = (T0_G * R_corr)

        if DHT:
            return R_corr # Not actually a real DH Transform 

        return T_total

    #
    # Forward Kinematics for arm
    #

    def do_FK(self, thetas, debug=False):
        t = self.getT(8, t1=thetas[0], t2=thetas[1], t3=thetas[2], t4=thetas[3], t5=thetas[4], t6=thetas[5])

        px = t[0,3]
        py = t[1,3]
        pz = t[2,3]

        roll, pitch, yaw = tf.transformations.euler_from_matrix(np.array(t[:3,:3]).astype(np.float64), axes='sxyz')

        # print "FK px py pz r p y", px, py, pz, roll, pitch, yaw

        return px, py, pz, roll, pitch, yaw
        
    #
    # Inverse Kinematics for arm
    #

    def do_IK(self, px, py, pz, roll, pitch, yaw, last_thetas=None, debug=False):

        if debug:
            print "---------------------------------------------------"
            print "px,   py,    pz  is", px, py, pz
            print "roll, pitch, yaw is", roll, pitch, yaw

        #################################
        # Back calculate wrist position
        #################################

        R_rpy = self.rot_z(yaw)*self.rot_y(pitch)*self.rot_x(roll) # Extrinsic r, p, y

        # debug and experimentation code
        # R_rzyx = tf.transformations.euler_matrix(yaw, pitch, roll, "rzyx")
        # print "R_rpy", R_rpy
        # print "R_rpy", R_rpy
        # print "R_sxyz", R_sxyz
        # print "R_rxyz", R_rxyz
        # print "R_szyx", R_szyx
        # print "R_rzyx", R_rzyx

        R0_6 = R_rpy

        # print "R0_6:"
        # pprint(R0_6)

        T_ypr = np.vstack((np.hstack((R0_6, np.matrix([[px], [py], [pz]]))), np.matrix([0.0, 0.0, 0.0, 1.0])))

        # print "T_ypr is"
        # pprint(T_ypr)

        # calculate wrist center as -wrist_length along the x axis
        wc = T_ypr * np.matrix([-self.wrist_length, 0.0, 0.0, 1.0]).T

        print "Wrist center:"
        pprint (wc)

        theta1 = 0.0
        theta2 = 0.0
        theta3 = 0.0
        theta4 = 0.0
        theta5 = 0.0
        theta6 = 0.0

        ####################################
        # Calculate First three joint angles
        ####################################

        # theta1 is set to the angle needed to turn the
        # arm towards the wrist center by default.  We try to face
        # the big join2 near the wrist center.
        # But if reaching over the back of the base (big joint
        # away from the wrist center) pevents us from having
        # to spin the base we will try that.
        # The danger is that we may not be able to reach the
        # target backwards, but we could if we spun the base.
        # This code does not cope with that. If reaching backwareds
        # prevents a base spin, we try it, and if we can't reach
        # the correct base position, the code just blows up.
        # For this class exercise, there are test cases where
        # it needs to reach backwards over the center, but there
        # are no cases where that backwards reach goess to far that
        # know of. So this should be good enough.
        # The "correct" way to code this is to find all possible
        # combinations of all 6 joints that can reach the given pose
        # and gripper location, and then pick from that list based
        # what's closest to the last position.  But I did not code that
        # for all 6 joints.

        theta1 = np.arctan2(wc[1,0], wc[0,0])       ## The simple one
        reach_back = False              # do we reach back over the center?

        # Disabled this code -- it works, but it allows the base to be
        # twisted around backwards to prevent a rotate, and then runs in to
        # the problem of not being able to finish the move by reaching "backwards"
        # far enough IN this exercise.  Better to just let it flip the base when the arm passes
        # overhead than to run out of reach.

        if 0 and last_thetas is not None:
            # Pick the theta1 that is the closest to the last theta used
            # This allows the arm to reach up and over without spining the base around.
            t1b = theta1 + np.pi
            if not self.joint1_in_range(t1b):
                t1b = theta1 - np.pi
            if self.joint1_in_range(t1b):
                # Ok, we have a good option to check
                # It's not always possible for there to be a second option
                if abs(t1b-last_thetas[0]) < abs(theta1-last_thetas[0]):
                    # The backwards reach is closer to the last one
                    if 0:
                        print "----------We are flipping theta1!!!!----------------"
                        print "from", theta1, "to", t1b
                    theta1 = t1b
                    reach_back = True

        # print "theta1 is", self.r_to_d(theta1), "degres"

        T2 = self.getT(2, t1=theta1, t2=0.0)
        o2 = T2 * np.matrix([0.0, 0.0, 0.0, 1.0]).T

        # print "T2"
        # pprint(T2)

        # print "o2:"
        # pprint(o2)

        # Look at the triangle formed by link2 and link3 and the
        # imaginary line that closes this triangle running from
        # joint2 to joint4 (aka the wrist center)

        # There are two ways the robot can bend it's "elbow" to
        # make this work -- we only code for it being in the orrientation
        # where joint 3 is high, and link 3 bends down to reach wc.

        # Find the length of the three sides of the triangle
        # The lenth of the two links are physical constants from the machine.
        # The third is computed since we know the location of o2 now, and
        # and the location of of the wrist center.

        l34 = np.sqrt(.054**2 + 1.5**2) # Straight line length from O3 to O4
        l23 = 1.25
        l24 = np.sqrt((wc[0,0]-o2[0,0])**2 + (wc[1,0]-o2[1,0])**2 + (wc[2,0]-o2[2,0])**2)

        if l24 > l23 + l34:
            print "ERROR -- location too far away to reach"
            l24 = (l23 + l34) * 0.99999 # force a slight round down to avoid problems

        # print "tri sides (l34, l23, l24)", l34, l23, l24

        # Use law of cosines to calculate angles from length of sides

        o3a = np.arccos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
        o2a = np.arccos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
        o4a = np.pi - o3a - o2a

        # print "triangles are", self.r_to_d(o3a), self.r_to_d(o2a), self.r_to_d(o4a)

        # Calculate angle from horozontal of imiginary third side which is
        # running from joint 2, to the wrist center.

        # Horizontal distance from wrist center, to z axes of orign (center of base)
        wc_to_0 = np.sqrt(wc[0,0]**2 + wc[1,0]**2)
        if reach_back:
            wc_to_0 = - wc_to_0 # Yes, this is complex
        # Horizontal distance from o2, to z axes of orign (center of base)
        o2_to_0 = np.sqrt(o2[0,0]**2 + o2[1,0]**2)
        # Angle from o2 to wc which will go negative if wc is inside rotaion circle
        # of o2.
        o2o4horz = np.arctan2(wc[2,0]-o2[2,0], wc_to_0 - o2_to_0)

        # use the joint 2 angle of the triangle, and the angle of joint 2
        # from horizontal, to compute the link angle, and adjust it so that
        # the linke is pointing straight up, it's 0, to find theta2

        theta2 = np.pi/2 - (o2o4horz + o2a)             # straight up is zero for robot

        # print "Theta2 is", self.r_to_d(theta2), "which is pi/2 - o2a - o2o4horz", self.r_to_d(o2a), self.r_to_d(o2o4horz)

        # Now that we know theta2 (and q2) use FK to compute location of o3

        T2 = self.getT(2, t1=theta1, t2=theta2)

        # print "q2 is", self.r_to_d(theta2-np.pi/2)

        o3 = T2*np.matrix([1.25, 0.0, 0.0, 1]).T

        # print "o3 is:"
        # pprint(o3)

        # We now know the world coordinates for the location of o3 and the wrist center wc (o4)

        # The angle of the triangle computed above tells us the angle of the joint3, except
        # Due to an offset in the arm from joint 3 to joint 4, the angle we must set theta3
        # has a constant angluar offset we must adjust for.

        # So first, calcuate the angular offset we need based on physical arm numbers.
        
        o3o4offset = np.arctan2(0.054, 1.5)

        theta3 = np.pi/2 - (o3a + o3o4offset)
        
        # print "o3o4offset angle is", self.r_to_d(o3o4offset), "theta3 is", self.r_to_d(theta3)

        # Use FK to find o4 location

        T3 = self.getT(3, t1=theta1, t2=theta2, t3=theta3)
        o4 = T3*np.matrix([-0.054, 1.5, 0.0, 1]).T

        # print "o4 is", o4
        # print "wc (should be same as 04)", wc

        if 0:
            # just more debug and test code
            # compute angles from the joint locations we found using FK
            theta1o2 = np.arctan2(o2[1], o2[0])
            theta1o3 = np.arctan2(o3[1], o3[0])
            theta1o4 = np.arctan2(o4[1], o4[0])
            theta1wc = np.arctan2(wc[1], wc[0])

            print "theta1 is", theta1, "and theta1o4,o2,wc,o3", theta1o4, theta1o2, theta1wc, theta1o3

            # compute lengths for testing

            # What's the distance from o3 to o4?  and o3 to wc?
            # o3o4distance = np.sqrt((o3[0]-o4[0])**2 + (o3[1]-o4[1])**2 + (o3[2]-o4[2])**2)
            o3o4distance = self.distance(o3, o4)
            print "distance from o3 to o4 is", o3o4distance, "should be same as l34:", l34, "which should be a little bit larget than 1.5"
            #o3wcdistance = np.sqrt((o3[0]-wc[0])**2 + (o3[1]-wc[1])**2 + (o3[2]-wc[2])**2)
            o3wcdistance = self.distance(o3, wc)
            print "distance from o3 to wc is", o3wcdistance, "should be same as l34:", l34, "which should be a little bit larget than 1.5"

            print "distance from o2 to o3", self.distance(o2, o3)
            print "distance from o2 to wc", self.distance(o2, wc)
            print "distance from o2 to o4", self.distance(o2, o4)
            print "distance from 000 to o2", self.distance(np.matrix([0,0,0]).T, o2)
            print "distance from .35/.75", np.sqrt(.35**2 + .75**2)

        if 0:
            # More debug and test
            # OK, calculate the triangle angle and sides again using o2, o3, and o4
            l23 = self.distance(o2, o3)
            l34 = self.distance(o3, o4)
            l24 = self.distance(o2, o4)
            print "NEW tri sides (l34, l23, l24)", l34, l23, l24
            o3a = np.arccos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
            o2a = np.arccos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
            o4a = np.pi - o3a - o2a
            print "NEW triangles are", self.r_to_d(o3a), self.r_to_d(o2a), self.r_to_d(o4a)

        #############################################
        # Now find q4 q5 and q6 for the end effector
        #############################################

        # To find the last three wrist angles, we no longer
        # worry about lenghts, only angles to create
        # right final pose of the wrist.

        # Find rotation from joint 3 to the world frame
        # by starting with the R_rpy total rotation matrix we
        # computed from r p y values, then factoring out
        # the rotation of the first three joints by using
        # the FK roation for R0_3.

        R3_w = T3.transpose()[:3,:3] * R_rpy

        # print "R3_w"
        # pprint(R3_w)
        # print "R3_w*self.rot_z(180)*self.rot_y(-90)"
        # pprint(R3_w*self.rot_z(180)*self.rot_y(-90))

        # R3_w now translates from link3 local frame (z along gripper,  x up) to
        # the gripper frame, expressed in world frame axis convention of z up
        # and x down gripper axis.

        # To issolate only the rotation from link 3 to the gripper, we must
        # first remove the axis Rotations from the matrix.
        # The z and x rotation sequence below moves from linke 3 axes to world
        # axis.  So we transpose (invert for R) to remove them from R3_w

        R3_6 = R3_w * (self.rot_z(np.pi/2) * self.rot_x(np.pi/2)).T

        # R3_6 is now the needed wrist rotations, in terms of the
        # link3 frame of x up and y forward.

        # print "R3_6"
        # pprint(R3_6)

        # Use the tf.trnsformaion routines to extract the needed joint angles
        # from the matrix.  We use "r" (intrinsic mode") since our joinst will
        # accumulate roations.  And we use y because that's the axis of joint
        # 4, z -- the axis of joint 5, and y again, the axis of joint 6.

        # Extract alpha, beta, and gamma Euler angles

        a, b, g = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), axes='ryzy')

        if 0:
            print "using", t, "abg from R3_6 is", a2, b2, g2

        #####################################################################################
        # If we know the last position, check all possible wrist solutions to see which is
        # the closest to the last solution.
        # theta4 and 6 can spin +- 350 but euler_from_matrix will only use +- 180
        # We can take advantage of this extra twisting range to reduce the odds of a required
        # wrist flip.
        #####################################################################################

        if last_thetas is not None:
            last_a = last_thetas[3]
            last_b = last_thetas[4]
            last_g = last_thetas[5]

            a, b, g = self.find_best_wrist(a, b, g, last_a, last_b, last_g)

        theta4 = a
        theta5 = b
        theta6 = g

        #########################################
        # Verify we got a valid answer!
        #########################################

        tt = self.getT(7, t1=theta1, t2=theta2, t3=theta3, t4=theta4, t5=theta5, t6=theta6)
        ttr = self.getT(8, t1=theta1, t2=theta2, t3=theta3, t4=theta4, t5=theta5, t6=theta6)

        tt_end = tt * np.matrix([0,0,0,1]).T

        if 0:
            tt_wc = ttr * np.matrix([-self.wrist_length,0,0,1]).T
            print "tt is", np.array(tt).astype(np.float64)
            print "tt_end is", tt_end.T
            print "tt_wc is", tt_wc.T
            print "distance from tt_end to wc is", self.distance(wc, tt_end)
            print "distance from tt_end to pxyz is", self.distance(tt_end, np.matrix([px, py, pz]).T)

        Rtt = np.array(ttr[:3,:3]).astype(np.float64)

        # print "Rtt is", Rtt

        end_xyz = np.array(tt_end[:3,:]).astype(np.float64).flatten()
        if not np.allclose(end_xyz, [px, py, pz]):
            print "ERROR -- final gripper location fails to match target"
            print "  target px, py, pz", px, py, pz
            print "  result px, py, pz", end_xyz[0], end_xyz[1], end_xyz[2]

        r,p,y = tf.transformations.euler_from_matrix(Rtt)
        if not np.allclose([roll, pitch, yaw], [r, p, y]):
            print "ERROR -- final gripper pose fails to match target"
            print "  target roll pitch yaw:", roll, pitch, yaw
            print "  result roll pitch yaw:", r, p, y

        #########################################
        # Track end location error
        #########################################

        if self.error_data is not None:
            self.error_data.append(self.tuple_distance(end_xyz, [px, py, pz]))

        if debug:
            print "joint angles: %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f" % (theta1, theta2, theta3, theta4, theta5, theta6)
        
        return theta1, theta2, theta3, theta4, theta5, theta6
        
    #
    # find_best_wrist() -- find the best wrist postion based on last position
    #
    # Uses a,b,g calucated by tf which are limited to +- 180 to find alternative
    # angles that will reduce the need for 180 deg filps.  joint 4, and 6 have
    # a range of +-350 deg so by making use of this larger range some need to rotate
    # the wrist can be eliminated.
    #
    # The theory here, is that the axies of the wrist (joint 5) is correct and in the position it
    # must be in for all solutions. But, it can be rotated a full 180 to flip the top
    # and bottom of the wrist.  So to flip joint 5 upside down, we rotate joint 4 and 5 180 deg, and
    # set joint 5 to the negative of it's angle.  And we can also rotate the last joint, 360 degrees
    # from this starting position in any directin that keeps it within it's +350 -305 range.
    #
    # So, we iterate through all combination of angles that are compatible with the given a,b,g
    # solution and find the one with the least amount of change from the last.  But, the concept
    # of "best" here is complex with no single right answer.
    #

    def find_best_wrist(self, a, b, g, last_a, last_b, last_g):

        best_dist = None
        best_a = None
        best_b = None
        best_g = None

        for i in range(-3, 4): # iterate through all possible +-pi wrist flips for joint 4
            new_a = a + i * np.pi
            if not self.joint4_in_range(new_a):
                # Outside of joint rotation range -- skip it
                continue
            new_b = b
            new_g = g
            g_offset = 0
            if i % 2 == 1: # odd number of pi
                new_b = - new_b
                # we need to flip g +- pi as well
                g_offset = np.pi
            for j in range(-2, 3): # iterate through all possible +-2*pi wrist flips for joint 6
                new_g = g + j * 2 * np.pi + g_offset
                if not self.joint6_in_range(new_g):
                    # Outside of joint rotation range -- skip it
                    continue

                # new_a, new_b, new_g is a possible solution
                # How close is new_a to the old?
                # Try sum of total rotations of all 3 joints as measure of best

                max_dist = max([abs(new_a-last_a), abs(new_b-last_b), abs(new_g-last_g)])
                sum_dist = abs(new_a-last_a) + abs(new_b-last_b) + abs(new_g-last_g)

                if 0:
                    print "ij %3d %3d   abj %6.3f %6.3f %6.3f   sum dist %6.3f" % (i, j, new_a, new_b, new_g, sum_dist),
                    if np.allclose([a, b, g], [new_a, new_b, new_g]):
                       print "starting default",
                    print

                dist = sum_dist

                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_a = new_a
                    best_b = new_b
                    best_g = new_g

        if best_dist is not None and not np.allclose((a, b, g), (best_a, best_b, best_g)):
            if 0:
                print "Using %6.3f %6.3f %6.3f instead of %6.3f %6.3f %6.3f" % (best_a, best_b, best_g, a, b, g),
                print " total wrist roations %6.3f" % best_dist,
                if best_a < np.pi < 2 or best_a > np.pi/2:
                    print " ALPHA extended",
                if best_g < np.pi < 2 or best_g > np.pi/2:
                    print " GAMMA extended",
                print

            return best_a, best_b, best_g
        
        if 0:
            print " total wrist roations %6.3f" % best_dist

        return a, b, g # Couldn't find anything better


    # Distance between two 3D points
    def distance(self, p1, p2):
        return (np.sqrt((p1[0,0]-p2[0,0])**2 + (p1[1,0]-p2[1,0])**2 + (p1[2,0]-p2[2,0])**2))

    def tuple_distance(self, p1, p2):
        return (np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2))

    # Radians to Degrees for debuging
    def r_to_d(self, x):
        return x * 180.0 / np.pi

    def rot_x(self, q):
        R_x = np.matrix([[1, 0, 0],
                    [0, np.cos(q), -np.sin(q)],
                    [0, np.sin(q), np.cos(q)]])
        
        return R_x
        
    def rot_y(self, q):              
        R_y = np.matrix([[np.cos(q), 0, np.sin(q)],
                    [0, 1, 0],
                    [-np.sin(q), 0, np.cos(q)]])
        
        return R_y

    def rot_z(self, q):    
        R_z = np.matrix([[np.cos(q), -np.sin(q), 0],
                    [np.sin(q), np.cos(q), 0],
                    [0, 0, 1]])
        
        return R_z

    def rot_x_sympy(self, q):
        R_x = Matrix([[1, 0, 0],
                    [0, cos(q), -sin(q)],
                    [0, sin(q), cos(q)]])
        
        return R_x
        
    def rot_y_sympy(self, q):              
        R_y = Matrix([[cos(q), 0, sin(q)],
                    [0, 1, 0],
                    [-sin(q), 0, cos(q)]])
        
        return R_y

    def rot_z_sympy(self, q):    
        R_z = Matrix([[cos(q), -sin(q), 0],
                    [sin(q), cos(q), 0],
                    [0, 0, 1]])
        
        return R_z


Robot = None

## Test code to help another student debug their code
##
def get_wrist_position(p, quant, info):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quant)
    print "get_wrist_position info is", info
    l_test(quant, roll, pitch, yaw, p[0], p[1], p[2])

    print "Call do_IK"
    Robot.do_IK(p[0], p[1], p[2], roll, pitch, yaw)
    print "------------ get wrist done"

def l_test(quant, r,p,y, p_x,p_y,p_z):
    R0_G = tf.transformations.quaternion_matrix(quant) #given a input
    re_orient=Matrix([[-1, 0, 0,  0],
    [ 0, -1, 0, 0],
    [ 0, 0,  -1, 0],
    [ 0, 0, 0,  1]])

    # Rrpy=R0_G*re_orient
    Rrpy=R0_G

    print "Rrpy", Rrpy
    print 


    global Robot
    curt_rpy = Robot.rot_z(y)*Robot.rot_y(p)*Robot.rot_x(r) # Extrinsic r, p, y
    print "curt rpy", curt_rpy


    l_x = Rrpy[0,0]
    l_y = Rrpy[1,0]
    l_z = Rrpy[2,0]

    #print(l_x,l_y,l_z)
    eel=0.303
    d6=0

    if 0:
        p_x=P_G[0]
        p_y=P_G[1]
        p_z=P_G[2]

    p_wc_x=p_x-(d6+eel)*l_x
    p_wc_y=p_y-(d6+eel)*l_y
    p_wc_z=p_z-(d6+eel)*l_z

    print "pwc", p_wc_x, p_wc_y, p_wc_z
    

def handle_calculate_IK(req):
    global Robot
    global Last_position
    global Last_thetas

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    if len(req.poses) < 1:
        print "No valid poses received"
        return CalculateIKResponse([]) # try return with empty list
        # return -1

    # Initialize service response
    joint_trajectory_list = []
    last_thetas = None
    last_position = None
    t0 = time.clock()

    do_error_plot = False
    
    if do_error_plot:
        Robot.start_error_plot()

    for x in xrange(0, len(req.poses)):
        # IK code starts here

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


        if last_thetas is None and Last_thetas is not None:
            # First position of this request.
            # If the position of this request is near to the last position of the last
            # request, we assume the wrist is still in the same orrientation (only a valid
            # idea for this class project, not good in general), and if so, we use the
            # last_thetas from the previous request to help prevent unnesaary wrist flips
            # for this move.  This ofen happens in this project aftr the arm has made the move
            # to grab the blue rod, but before it moves to the drop off bin.  This prevents
            # a wrist flip at the start of the move to the bin -- which then causes the wrist
            # flip to "unwind" the twisted wrist to happen after the drop off, when it's swining
            # back to reset to the start.  Which I like better, than flipping while it's holding
            # the blue rod!
            # print "Distance from last known location is", Robot.tuple_distance(Last_position, (px, py, pz))
            if Robot.tuple_distance(Last_position, (px, py, pz)) < 0.1:
                # Wrist starting position is near the end of the last move, so we assume
                # the wrist orriention is the same as we left it last time (true for this project)
                last_thetas = Last_thetas
                # print "Using Last_thetas to help optimize first move of this request!"
        theta1, theta2, theta3, theta4, theta5, theta6 = Robot.do_IK(px, py, pz, roll, pitch, yaw, last_thetas=last_thetas)

        rospy.loginfo("IK joint angles: %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f" % (theta1, theta2, theta3, theta4, theta5, theta6))

        if 0: # help another student
            l_test([req.poses[x].orientation.x, req.poses[x].orientation.y,
                            req.poses[x].orientation.z, req.poses[x].orientation.w], roll, pitch, yaw, px, py, pz)
            get_wrist_position([1.5,-0.3,1.04],[-0.299,-0.322,-0.635,0.634], "[1.3,-0.3,0.8] and it should be [1.50,0,1.038]")

        last_position = (px, py, pz)
        last_thetas = (theta1, theta2, theta3, theta4, theta5, theta6)

        # print "thetas are", theta1, theta2, theta3, theta4, theta5, theta6

        # Populate response for the IK request

        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

    if do_error_plot and len(Robot.error_data) > 100:
        Robot.show_error_plot() # will hang process until plot window is closed

    Last_position = last_position
    Last_thetas = last_thetas

    t1 = time.clock()
    steps = len(joint_trajectory_list)

    rospy.loginfo("total IK time %4.1f msec for %d steps, %5.3f msec per step" % ((t1-t0)*1000.0, steps, (t1-t0)*1000.0/steps))
    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # Init robot arm
    global Robot
    Robot = Kuka_KR210()
    print "return for init"

    global Last_position
    Last_position = None
    global Last_thetas
    Last_thetas = None

    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()




def test_IK(px, py, pz, roll, pitch, yaw, debug=True):
            if debug:
                print "---------------------------------------------------"
                print "px,   py,    pz  is", px, py, pz
                print "roll, pitch, yaw is", roll, pitch, yaw
            # Define DH param symbols
            # link_offset_i: signed distance from x_(i-1) to x_i along z_i
            # link_length_(i-1): distance from z_(i-1) to z_i along x_i, where x_i is perpendicular to both z_(i-1) and z_i
            # twist_angle_(i-1): angle between z_(i-1) and z_i about x_(i-1) in the right hand sense
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link_offset_i
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link_length_(i-1)
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')# twist_angle_(i-1)
            
            # Joint angle symbols
            # theta_i: joint angle between x_(i-1) and x_i about z_i in the right hand sense
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i

            # Modified DH params
            s = {alpha0:     0,   a0:      0,   d1:  0.75,
                 alpha1: -pi/2,   a1:   0.35,   d2:     0,   q2: q2-pi/2,
                 alpha2:     0,   a2:   1.25,   d3:     0,
                 alpha3: -pi/2,   a3: -0.054,   d4:   1.5,
                 alpha4:  pi/2,   a4:      0,   d5:     0,
                 alpha5: -pi/2,   a5:      0,   d6:     0,
                 alpha6:     0,   a6:      0,   d7: 0.303,   q7:       0}

            # Create individual transformation matrices
            T0_1 = Transformation_Matrix(q1, alpha0, a0, d1)
            T0_1 = T0_1.subs(s)

            T1_2 = Transformation_Matrix(q2, alpha1, a1, d2)
            T1_2 = T1_2.subs(s)

            T2_3 = Transformation_Matrix(q3, alpha2, a2, d3)
            T2_3 = T2_3.subs(s)
            
            if 0:
                # Extract end-effector position and orientation from request
                # px,py,pz = end-effector position
                # roll, pitch, yaw = end-effector orientation
                px = req.poses[x].position.x
                py = req.poses[x].position.y
                pz = req.poses[x].position.z

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                        req.poses[x].orientation.z, req.poses[x].orientation.w])
            
            # Intrinsic rotation correction for end-effector transformation matrix 
            R_z_intrinsic = Matrix([[    cos(pi),    -sin(pi),               0],
                                    [    sin(pi),     cos(pi),               0],
                                    [          0,           0,               1]])
            R_y_intrinsic = Matrix([[ cos(-pi/2),           0,      sin(-pi/2)],
                                    [          0,           1,               0],
                                    [-sin(-pi/2),           0,      cos(-pi/2)]])

            ZY_intrinsic_rot = R_z_intrinsic * R_y_intrinsic

            # Calculate transformation matrix from base link to end-effector
            endeffector_trans = Matrix([[px],
                                        [py],
                                        [pz]])

            R_z_extrinsic = Matrix([[   cos(yaw),   -sin(yaw),               0],
                                    [   sin(yaw),    cos(yaw),               0],
                                    [          0,           0,               1]])
            R_y_extrinsic = Matrix([[ cos(pitch),           0,      sin(pitch)],
                                    [          0,           1,               0],
                                    [-sin(pitch),           0,      cos(pitch)]])
            R_x_extrinsic = Matrix([[          1,           0,               0],
                                    [          0,   cos(roll),      -sin(roll)],
                                    [          0,   sin(roll),      cos(roll)]])

            XYZ_extrinsic_rot = R_z_extrinsic * R_y_extrinsic * R_x_extrinsic
            R0_6_rotation = XYZ_extrinsic_rot * ZY_intrinsic_rot
            R0_6 = R0_6_rotation.row_join(endeffector_trans)
            bottom_row = Matrix([[0, 0, 0, 1]])
            R0_6 = R0_6.col_join(bottom_row)
     
            # Find Wrist Center Location
            d_7 = s[d7]

            print "R0_6 is"
            pprint(R0_6)
            print "using 0,0,for x which is", R0_6[0,0]
            print "using 1,0,for y which is", R0_6[1,0]
            print "using 2,0,for z which is", R0_6[2,0]

            wx = px - (d_7 * R0_6[0, 2])
            wy = py - (d_7 * R0_6[1, 2])
            wz = pz - (d_7 * R0_6[2, 2])

            # Finding theta 1-3
            a_3 = s[a3]
            d_4 = s[d4]
            d_1 = s[d1]
            a_1 = s[a1]
            a_2 = s[a2]

            s4 = sqrt(a_3**2 + d_4**2)
            s3 = sqrt((wz - d_1)**2 + (wx - a_1)**2)
            beta1 = atan2((wz - d_1), (wx - a_1))
            beta2 = acos((a_2**2 + s3**2 - s4**2) / (2 * a_2 * s3))
            beta3 = acos((a_2**2 + s4**2 - s3**2) / (2 * a_2 * s4))
            beta4 = atan2(a_3, d_4)

            theta1 = atan2(wy, wx).evalf()
            theta2 = ((pi / 2) - beta2 - beta1).evalf()
            theta3 = ((pi / 2) - beta3 - beta4).evalf()

            # Finding theta 4-6
            R0_3 = simplify(T0_1 * T1_2 * T2_3)
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R0_3_inv = R0_3.inv()
            R3_6 = R0_3_inv * R0_6
            
            r11 = R3_6[0, 0]
            r21 = R3_6[1, 0]
            r31 = R3_6[2, 0]
            r32 = R3_6[2, 1]
            r33 = R3_6[2, 2]

            theta4 = atan2(r21, r11).evalf()
            theta5 = atan2(-r31, sqrt(r11 ** 2 + r21 ** 2)).evalf()
            theta6 = atan2(r32, r33).evalf()

            # wx = wx.evalf()
            # wy = wy.evalf()
            # wz = wz.evalf()

            print "distance from wrist center to px,py,pz:"
            print sqrt((wx-px)**2 + (wy-py)**2 + (wz-pz)**2)


            print('wx: ' + str(wx.evalf()))
            print('wy: ' + str(wy.evalf()))
            print('wz: ' + str(wz.evalf()))
            print('theta1: ' + str(theta1.evalf()))
            print('theta2: ' + str(theta2.evalf()))
            print('theta3: ' + str(theta3.evalf()))
            print('theta4: ' + str(theta4.evalf()))
            print('theta5: ' + str(theta5.evalf()))
            print('theta6: ' + str(theta6.evalf()))


            return theta1, theta2, theta3, theta4, theta5, theta6

def Transformation_Matrix(q, alpha, a, d):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    return(T)

if __name__ == "__main__":
    IK_server()
