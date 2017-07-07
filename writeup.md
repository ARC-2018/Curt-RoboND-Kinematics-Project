## Project: Kinematics Pick & Place
### Student Writeup by Curt Welch <curt@kcwc.com>
### July 7, 2017
### Project submited as a github repository
### https://github.com/curtwelch/RoboND-Kinematics-Project

---

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---
### Metrics

### 1. Provide a Writeup 

You're reading it!

### 2. Kinematic Analysis
#### 2-1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

A hand-drawn diagram was suggested, so here's the hand-drawn notes I created for the project!

![DH Hand Drawing](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/DH%20drawing.jpg)

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

#### 2-2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


##### 2-3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


### 3. Project Implementation

#### 3-1. Fill in the `IK_server.py`

My file can be found in my repository here: [IK_server.py](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/IK_server.py)

As submited the code is configured to run as an IK server, but it includes a good bit of addition test and and debug features that are either commented out or turned off.

The Kinematics code for the arm was all moved into a class called Kuka_KR210.  The early version of the code used sympy as per the examples in the lessons, but I got tired of debuging code that was running so slow so I rewrote all the math to use numpy awhich made it run about 300 times faster.  The sympy version of the transformation code is still to be found in the `getT_sympy()` method which was also used above to produce the symbolic versions of the Transfrom tables for this write-up.  But the bulk of the rest of the code is all based on numpy.

The `do_FK()` method computes forward kinematics by taking a set of 6 joint angles and computing the end effector x, y, and z location and it's pose in the form of row, pitch, and yaw values.  This uses the DH transformation matrices created by the `getT()` method.

The `do_IK()` method is the Inverse Kinematcis code in numpy.

The IK process works roughly as follows.  A wrist center location in the world frame is calculated using `px, py, pz` and `roll, pitch, yaw` input values. Theta1, theta2, and theta3 are computed to move the arm's wrist to this calucated wrist center.  Theta1 rotates the arm's vertical motion frame to intersect with the wrist center such that joint 2, the large lower "sholder" joint is positioned towards the wrist center. Link2 and Link3 form a virtual triangle from joint 2 to the wrist center, and since we know the length of the sides of this triangle we can calucate all the angles to compute theta2 and theat3 using straight forward trig along with known arm geometry values.

Once theta1, theta2, and theta3 re computed, FK is used to compute a rotation matrix for links 0 to 3.  Using row, pitch, and yaw values we can also compute the gripper's roation. These two are then used to compute a gripper roation matrix relative to link3.  This is a little tricky due do the fact that the axis orrienations flip in this process so axis flips are also adjusted in the process to produce a roation matix that only tells us how the griper is rotated relative to link3 of the arm.

Wrist rotation values for theta4, theta5, and theta6 are computed using the tf.transformaions routine directly from the rotation matrix created.

##### Wrist Flips and Alternate Arm Positions

For any given IK problem defined by gripper locaiton and pose, there are many different arm configurations that can move the gripper to the correct location and angle.  The base can be rotated towards, or away from the wrist center. and links 2 and 3, can form a triangle with the elbow riased, or lowered -- creating 4 possible configurations for theta1, theta2, and theta3 to position the arm at the correct wirst center.

The test exercise will sometimes produce paths that riase the arm up and over the base to make it reach backwards and then lower it again.  Without adjusting for alternate base configurations this path will force the base to do a 180 deg flip in the middle of the motion which is slow, time consuming, and forces a momentary wild off-path swing of the gripper as the base flips around.  I wrote code to consider the alternate base configuration where joint2 can be allowed to be on the far side of the base away from the wrist center and it worked fine to stop the inital base flip motion.  However, the paths generated often then caused the based to twist around and force the arm to reach backwards to the shelf -- which was not possible beuase the joints in the arms coulod not bend far ebnnough to reach the shelf bacwards like that.  So the base rotate was required anyway.  The more advanced code only changed where the base rotated, and was not able to produce paths without base roations due to the types of paths generated in this exercise.  So I didabled the base flip code in this version.

The same alternate arm configuation problem of the base, exists for the wrist as well. But it's much worse for the wrist due to the large rotation range of joint 4 and joint 5 (+- 350 degs).  There are often as many as 6 or 8 different wrist configurations to put the gripper into the correct orrientation.

This exercise generates paths for the gripper that are full of gripper rotations, and without care in picking the correct wrist configuration at each step, the gripper will often have to pause and do a full 360 flip to get from one point in the move to the next.

I wrote code to test all possible wrist configuations and pick the best one based on minimizing rotations from the previous pose.  Unlike the base flip code that didn't help improve the base flip problem in the paths of this exercise, the wrist orriention code greatly reduced the need for wrist flips in the paths.  They almost never happen now. The gripper is almost always able to move though the motions without need for wrist flips in the middle of the motion.  At the end of the pick and place, when the arm moves back to the start, it now has to "unwind" twists created by the pick and place motions. So you will notice a set of smooth roations and flips to get the arm back to the zero starting position as it moves back from the bin, to the starting location.

The wrist configuration optimization code can be found in the `find_best_wrist()` method.

##### End Effector Location Error Plot

Optional work suggested in lesson. This data includes about 2 1/2 pick and place cycles because it collected data from multiple server requests until it had over 100 points to plot. The values are the distance from the starting px,py,pz location to the value computed by running IK, then FK, to re-compute the same point. The limit of 64 bit floating point representation is about e-16 so these values just represent the accumulation of floating point rounding errors in the IK and FK computations.
  
![Error Plot](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/error%20plot.png)

