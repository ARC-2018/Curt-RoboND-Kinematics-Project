## Project: Kinematics Pick & Place
### Student Writeup by Curt Welch <curt@kcwc.com>
### July 7, 2017
### Project submited as a github repository
### https://github.com/curtwelch/RoboND-Kinematics-Project

---

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---
### Metrics

#### 1. Provide a Writeup 

You're reading it!

#### 2. Kinematic Analysis
##### 2-1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

A hand-drawn diagram was suggested, so here's the hand-drawn notes I created for the project!

![DH Drawing](misc_images/DH Drawing.jpg)

The Modified DH Table parameter table

i | Alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | t1
2 | -pi/2 | 0.35 | 0 | t2 - pi/2
3 | 0| 1.25 | 0 | t3
4 | -pi/2 | -0.054 | 1.5 | t4
5 | pi/2 | 0 | 0 | t5
6 | -pi/2 | 0 | 0 | t6
7 | 0 | 0 | 0.303 | 0


The table from my code:
```
s = {       alpha0: 0,      a0: 0,
            alpha1: -pi/2,  a1: 0.35,   d1: 0.75,
            alpha2: 0,      a2: 1.25,   d2: 0,
            alpha3: -pi/2,  a3: -0.054, d3: 0,
            alpha4: pi/2,   a4: 0,      d4: 1.5,
            alpha5: -pi/2,  a5: 0,      d5: 0,
            alpha6: 0,      a6: 0,      d6: 0,
                                        d7: 0.303,  q7: 0}
```
This table was created by starting with the joint locations and orientations suggested from the lesson for the KR210 arm.  The parameters were then derived from the the URDF file that described the arm geometry for Gazebo.  Since many DH joint locations are shifted from the location in the URDF file many of the dimensions had to be adjusted or computed from multiple values in the URDF file. In the lesson video, at least one of the parameters were mis-labeled (a1 I believe) so everything had to be double checked to verify they represented a valid DH configuration for this arm.  Writting code to compute forward kinematics from this table, and comparing to numbers from the simulator then verfied the table was valid.

Line 7 in my table is a fake joint used to establish a final frame of reference at the center of the gripper which is .303 m further down the arm from the DH location of joint 6 (which is placed at the wrist center).

##### 2-2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


##### 2-3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


### Project Implementation

#### 1. Fill in the `IK_server.py`


The file can be found in my repository here: [IK_server.py](kuka_arm/scripts/IK_server.py)
  



