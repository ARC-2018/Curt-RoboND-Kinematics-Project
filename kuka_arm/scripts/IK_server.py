#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Student: Curt Welch <curt@kcwc.com>
# July 1, 2017

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

class Kuka_KR210:
    def __init__(self):
        self.wrist_length = 0.303
        if 0:
            self.do_test()

    def do_test(self):
        # Test IK code
        if 1:
            # One real simple from runing the demo
            px,py,pz = 2.0900910021952077, 0.900061404046043, 2.345040102031809
            roll, pitch, yaw = -0.0006983567709816882, 0.0006055558123970211, -0.0008013688952962057
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)
        if 1:
            ## DH Home position -- angles all zero
            px,py,pz = 2.153, 0.0, 1.946
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)
        if 1:
            # test for guy on slack that broke my code
            # high test wc inside j2 circle
            print atan2(0.229042, 0.165865).evalf()
            px,py,pz = 0.165865+0.303, 0.229042, 2.5848
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)
        if 1:
            # Low test fixking above high test borke this?
            px,py,pz = 1.0+0.303, 1.0, 0.2
            roll, pitch, yaw = 0.0, 0.0, 0.0
            theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=True)

        print "Repeat last test without debug"
        theta1, theta2, theta3, theta4, theta5, theta6 = self.do_IK(px, py, pz, roll, pitch, yaw, debug=False)
        print "angles are", theta1, theta2, theta3, theta4, theta5, theta6

        sys.exit(0)


    def getR(self, n, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None):
        # Get rotation matrix for different arm frames, n=1 to 8
        r = self.getT(n, t1=t1, t2=td2, t3=t3, t4=t4, t5=t5, t6=t6)
        return r[:3,:3]

    def getT(self, n, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None):

        # Return Homogeneous transformation matrix for arm frames
        # n controls which frame matrix is returned (1 to 8)
        # rotation_only controls if only the rotation matrix is returned.
        # n=1 returns T0_1, n=2 returns T0_2, ... n=7 returns T0_G, n=8 returns T0_world
        # T0_G has griper hand at 0,0 with x up, z forward
        # T0_world is same as T0_G except axis rotated to world frame of z up, x forward

        if n == 0:
            return eye(4)

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        print "Calculate Forward Kinematic Matrix for n=", n
                
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
            s.update({q2: t2})
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
            return T0_1

        T1_2 = Matrix([ [cos(q2),              -sin(q2),            0,              a1],
                        [sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1), -sin(alpha1),   -sin(alpha1)*d2],
                        [sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1), cos(alpha1),   cos(alpha1)*d2],
                        [0,                     0,                  0,              1]])
        T1_2 = T1_2.subs(s).evalf()
        T0_2 = (T0_1 * T1_2).evalf()
        if n == 2:
            return T0_2

        T2_3 = Matrix([ [cos(q3),              -sin(q3),            0,              a2],
                        [sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2), -sin(alpha2),   -sin(alpha2)*d3],
                        [sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2), cos(alpha2),   cos(alpha2)*d3],
                        [0,                     0,                  0,              1]])
        T2_3 = T2_3.subs(s).evalf()
        T0_3 = (T0_2 * T2_3).evalf()
        if n == 3:
            return T0_3

        T3_4 = Matrix([ [cos(q4),              -sin(q4),            0,              a3],
                        [sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3), -sin(alpha3),   -sin(alpha3)*d4],
                        [sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3), cos(alpha3),   cos(alpha3)*d4],
                        [0,                     0,                  0,              1]])
        T3_4 = T3_4.subs(s).evalf()
        T0_4 = (T0_3 * T3_4).evalf()
        if n == 4:
            return T0_4

        T4_5 = Matrix([ [cos(q5),              -sin(q5),            0,              a4],
                        [sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4), -sin(alpha4),   -sin(alpha4)*d5],
                        [sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4), cos(alpha4),   cos(alpha4)*d5],
                        [0,                     0,                  0,              1]])
        T4_5 = T4_5.subs(s)
        T0_5 = (T0_4 * T4_5)
        if n == 5:
            return T0_5

        T5_6 = Matrix([ [cos(q6),              -sin(q6),            0,              a5],
                        [sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5), -sin(alpha5),   -sin(alpha5)*d6],
                        [sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5), cos(alpha5),   cos(alpha5)*d6],
                        [0,                     0,                  0,              1]])
        T5_6 = T5_6.subs(s)
        T0_6 = (T0_5 * T5_6)
        if n == 6:
            return T0_6

        T6_G = Matrix([ [cos(q7),              -sin(q7),            0,              a6],
                        [sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6), -sin(alpha6),   -sin(alpha6)*d7],
                        [sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6), cos(alpha6),   cos(alpha6)*d7],
                        [0,                     0,                  0,              1]])
        T6_G = T6_G.subs(s)
        T0_G = (T0_6 * T6_G)
        if n == 7:
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

        return T_total

    def do_IK(self, px, py, pz, roll, pitch, yaw, debug=False):

        print "---------------------------------------------------"
        print "px,   py,    pz  is", px, py, pz
        print "roll, pitch, yaw is", roll, pitch, yaw

        #################################
        # Back calculate wrist position
        #################################

        R_rpy = rot_z(yaw)*rot_y(pitch)*rot_x(roll) # Correct for rpy we are given

        # debug and experimentation code
        # R_rzyx = tf.transformations.euler_matrix(yaw, pitch, roll, "rzyx")
        # print "R_rpy", R_rpy
        # print "R_rpy", R_rpy
        # print "R_sxyz", R_sxyz
        # print "R_rxyz", R_rxyz
        # print "R_szyx", R_szyx
        # print "R_rzyx", R_rzyx

        if 0: # experiments
            for t in "rxyz rzyx sxyz szyx".split():
                print " Transform", t
                r,p,y = tf.transformations.euler_from_matrix(np.array(R_rpy).astype(np.float64), axes=t)
                print t, "on R_rpy are", r, p, y

        R0_6 = R_rpy

        # print "R0_6:"
        # pprint(R0_6)

        T_ypr = R0_6.row_join(Matrix([px, py, pz])).col_join(Matrix([[0,0,0,1]]))

        # print "T_ypr", T_ypr

        # calculate wrist center as -wrist_length along the x axis
        wc = T_ypr * Matrix([-self.wrist_length, 0.0, 0.0, 1.0])

        # print "Wrist center:"
        # pprint (wc)

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
        # arm towards the wrist center.  We are assuming
        # big joint2 will always be orriented nearer to
        # the wrist center.  The robot can spin around 180
        # and reach back to grab it, but we don't ever use
        # That configuration in this code.

        theta1 = atan2(wc[1], wc[0]).evalf() ## The simple one

        # print "theta1 is", self.r_to_d(theta1), "degres"

        o1 = Matrix([0.0, 0.0, 0.75, 1.0])

        T2 = self.getT(2, t1=theta1, t2=0.0)
        o2 = (T2*Matrix([0,0,0,1])).evalf()

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

        l34 = sqrt(.054**2 + 1.5**2) # Straignt line length from O3 to O4
        l23 = 1.25
        l24 = sqrt((wc[0]-o2[0])**2 + (wc[1]-o2[1])**2 + (wc[2]-o2[2])**2)

        # print "tri sides (l34, l23, l24)", l34, l23, l24

        # Use law of cosines to calculate angles from length of sides

        o3a = acos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
        o2a = acos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
        o4a = (pi - o3a - o2a).evalf()

        # print "triangles are", self.r_to_d(o3a), self.r_to_d(o2a), self.r_to_d(o4a)

        # Calculate angle from horozontal of imiginary third side which is
        # running from joint 2, to the wrist center.

        # Horizontal distance from wrist center, to z axes of orign (center of base)
        wc_to_0 = sqrt(wc[0]**2 + wc[1]**2)
        # Horizontal distance from o2, to z axes of orign (center of base)
        o2_to_0 = sqrt(o2[0]**2 + o2[1]**2)
        # Angle from o2 to wc which will go negative if wc is inside rotaion circle
        # of o2.
        o2o4horz = atan2(wc[2]-o2[2], wc_to_0 - o2_to_0)

        # use the joint 2 angle of the triangle, and the angle of joint 2
        # from horizontal, to compute the link angle, and adjust it so that
        # the linke is pointing straight up, it's 0, to find theta2

        theta2 = (pi/2 - (o2o4horz + o2a)).evalf() # straight up is zero for robot

        # print "Theta2 is", self.r_to_d(theta2), "which is pi/2 - o2a - o2o4horz", self.r_to_d(o2a), self.r_to_d(o2o4horz)

        # Now that we know theta2 (and q2) use FK to compute location of o3

        T2 = self.getT(2, t1=theta1, t2=theta2-pi/2)

        # print "q2 is", self.r_to_d(theta2-pi/2)

        o3 = T2*Matrix([1.25, 0.0, 0.0, 1])

        # print "o3 is:"
        # pprint(o3)

        # We now know the world coordinates for the location of o3 and the wrist center wc (o4)

        # The angle of the triangle computed above tells us the angle of the joint3, except
        # Due to an offset in the arm from joint 3 to joint 4, the angle we must set theta3
        # has a constant angluar offset we must adjust for.

        # So first, calcuate the angular offset we need based on physical arm numbers.
        
        o3o4offset = atan2(0.054, 1.5).evalf()

        theta3 = (pi/2 - (o3a + o3o4offset)).evalf()
        
        # print "o3o4offset angle is", self.r_to_d(o3o4offset), "theta3 is", self.r_to_d(theta3)

        # Use FK to find o4 location

        T3 = self.getT(3, t1=theta1, t2=theta2-pi/2, t3=theta3)
        o4 = (T3*Matrix([-0.054, 1.5, 0.0, 1])).evalf()

        # print "o4 is", o4
        # print "wc (should be same as 04)", wc

        if 0:
            # just more debug and test code
            # compute angles from the joint locations we found using FK
            theta1o2 = atan2(o2[1], o2[0])
            theta1o3 = atan2(o3[1], o3[0])
            theta1o4 = atan2(o4[1], o4[0])
            theta1wc = atan2(wc[1], wc[0])

            print "theta1 is", theta1, "and theta1o4,o2,wc,o3", theta1o4, theta1o2, theta1wc, theta1o3

            # compute lengths for testing

            # What's the distance from o3 to o4?  and o3 to wc?
            # o3o4distance = sqrt((o3[0]-o4[0])**2 + (o3[1]-o4[1])**2 + (o3[2]-o4[2])**2)
            o3o4distance = self.distance(o3, o4)
            print "distance from o3 to o4 is", o3o4distance, "should be same as l34:", l34, "which should be a little bit larget than 1.5"
            #o3wcdistance = sqrt((o3[0]-wc[0])**2 + (o3[1]-wc[1])**2 + (o3[2]-wc[2])**2)
            o3wcdistance = self.distance(o3, wc)
            print "distance from o3 to wc is", o3wcdistance, "should be same as l34:", l34, "which should be a little bit larget than 1.5"

            print "distance from o2 to o3", self.distance(o2, o3)
            print "distance from o2 to wc", self.distance(o2, wc)
            print "distance from o2 to o4", self.distance(o2, o4)
            print "distance from 000 to o2", self.distance(Matrix([0,0,0]), o2)
            print "distance from .35/.75", sqrt(.35**2 + .75**2)

        if 0:
            # More debug and test
            # OK, calculate the triangle angle and sides again using o2, o3, and o4
            l23 = self.distance(o2, o3)
            l34 = self.distance(o3, o4)
            l24 = self.distance(o2, o4)
            print "NEW tri sides (l34, l23, l24)", l34, l23, l24
            o3a = acos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
            o2a = acos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
            o4a = (pi - o3a - o2a).evalf()
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

        # R3_w now translates from link3 local frame (z along gripper,  x up) to
        # the gripper frame, expressed in world frame axis convention of z up
        # and x down gripper axis.

        # To issolate only the rotation from link 3 to the gripper, we must
        # first remove the axis Rotations from the matrix.
        # The z and x rotation sequence below moves from linke 3 axes to world
        # axis.  So we transpose (invert for R) to remove them from R3_w

        R3_6 = (R3_w * (rot_z(pi/2) * rot_x(pi/2)).transpose()).evalf()

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

        theta4 = a
        theta5 = b
        theta6 = g

        if debug:
            tt = self.getT(7, t1=theta1, t2=theta2-pi/2, t3=theta3, t4=theta4, t5=theta5, t6=theta6)
            ttr = self.getT(8, t1=theta1, t2=theta2-pi/2, t3=theta3, t4=theta4, t5=theta5, t6=theta6)

            tt_end = tt * Matrix([0,0,0,1])
            tt_wc = ttr * Matrix([-self.wrist_length,0,0,1])

            if 0:
                print "tt is", np.array(tt).astype(np.float64)
                print "tt_end is", tt_end
                print "tt_wc is", tt_wc
                print "distance from tt_end to wc is", self.distance(wc, tt_end)
                print "distance from tt_end to pxyz is", self.distance(tt_end, Matrix([px, py, pz]))

            Rtt = np.array(ttr[:3,:3]).astype(np.float64)

            # print "Rtt is", Rtt

            # r,p,y = tf.transformations.euler_from_matrix(Rtt)
            # print "final rpy is", r, p, y

            #########################################
            # Verify we got a valid answer!
            #########################################

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
        
            print "angles are:", theta1, theta2, theta3, theta4, theta5, theta6

        return theta1, theta2, theta3, theta4, theta5, theta6

    # Distance between two 3D points
    def distance(self, p1, p2):
        return (sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)).evalf()

    # Radians to Degrees for debuging
    def r_to_d(self, x):
        return (x * 180.0 / pi).evalf()

Robot = None

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

            global Robot
            theta1, theta2, theta3, theta4, theta5, theta6 = Robot.do_IK(px, py, pz, roll, pitch, yaw)

            # print "thetas are", theta1, theta2, theta3, theta4, theta5, theta6

            # Populate response for the IK request

	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # Init robot arm
    global Robot
    Robot = Kuka_KR210()
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
