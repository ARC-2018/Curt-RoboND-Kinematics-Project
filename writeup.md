## Project: Kinematics Pick & Place
### Student Writeup by Curt Welch <curt@kcwc.com>
### July 7, 2017
### Project submitted as a github repository
### https://github.com/curtwelch/RoboND-Kinematics-Project

---

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---
### Metrics

### 1. Provide a Writeup 

Here it is!

### 2. Kinematic Analysis
#### 2-1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

A hand-drawn diagram was suggested, so here's the hand-drawn notes I created for the project!  It's not a clear image of course, but it was good enough to allow me to complete this project and it seems to meet the requirements for this rubric.

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

Transformation Matrices for the 6 joints that represent all the formal DH transformations:

![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T0_1.png)
![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T1_2.png)
![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T2_3.png)
![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T3_4.png)
![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T4_5.png)
![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T5_6.png)

The last DH "fake" transform to move the origin out to the center of the gripper to match the location given to us in the IK problem:

![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T6_7.png)

And one more rotation needed to correct axis orrientation from link 6 to the world frame (known as R_corr() in the lessons):

![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T7_8.png)

And now, the total transformation using px, py, pz, and roll, pitch, and yaw given to us in the IK problem:

![](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/T_total.png)

The first 8 transforms, when combined, using joint angles, will match the last transformation matrix, created using the position and pose of the gripper.

These images were created in the `print_report_information()` function you can find in my IK_server.py code and captured with a simple screen grab.

#### 2-3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The IK process works roughly as follows.  Far more code specific details are in the comments of the code than are here. This is just a rough overview.

A wrist center location in the world frame is calculated using `px, py, pz` and `roll, pitch, yaw` input values.

```
R_rpy = self.rot_z(yaw)*self.rot_y(pitch)*self.rot_x(roll) # Extrinsic r, p, y
R0_6 = R_rpy
T_ypr = np.vstack((np.hstack((R0_6, np.matrix([[px], [py], [pz]]))), np.matrix([0.0, 0.0, 0.0, 1.0])))
wc = T_ypr * np.matrix([-self.wrist_length, 0.0, 0.0, 1.0]).T
```

Theta1, theta2, and theta3 are computed to move the arm's wrist to this calucated wrist center.  Theta1 rotates the arm's vertical motion frame to intersect with the wrist center such that joint 2, the large lower "sholder" joint is positioned towards the wrist center.

```
theta1 = np.arctan2(wc[1,0], wc[0,0]) 
```

Link2 and Link3 form a virtual triangle from joint 2 to the wrist center, and since we know the length of the sides of this triangle we can calucate all the angles to compute theta2 and theat3 using straight forward trig (law of cosines) along with known arm geometry values.

Length of the three sides:

```
l34 = np.sqrt(.054**2 + 1.5**2) # Straight line length from O3 to O4
l23 = 1.25
l24 = np.sqrt((wc[0,0]-o2[0,0])**2 + (wc[1,0]-o2[1,0])**2 + (wc[2,0]-o2[2,0])**2)
```

Three angles of the triangle:

```
o3a = np.arccos((l24**2 - l23**2 - l34**2) / (-2*l23*l34))
o2a = np.arccos((l34**2 - l23**2 - l24**2) / (-2*l23*l24))
o4a = np.pi - o3a - o2a
```

One more angle needed to tell us the roation of the triangle relative to horizontal.  The angle about joint 2 from horizontal to the wrist center is computed:

```
wc_to_0 = np.sqrt(wc[0,0]**2 + wc[1,0]**2)
o2_to_0 = np.sqrt(o2[0,0]**2 + o2[1,0]**2)
o2o4horz = np.arctan2(wc[2,0]-o2[2,0], wc_to_0 - o2_to_0
```

And, yet another angle of the offset in the link3 between joint3 and joint 4:

```
o3o4offset = np.arctan2(0.054, 1.5)
```

Theta2, and theta3 computed from all the angles:

```
theta2 = np.pi/2 - (o2o4horz + o2a)
theta3 = np.pi/2 - (o3a + o3o4offset)      
```

Once theta1, theta2, and theta3 are computed, FK is used to compute a rotation matrix for links 0 to 3.  Using roll, pitch, and yaw values we can also compute the gripper's rotation. These two are then used to compute a gripper roation matrix relative to link3.  This is a little tricky due do the fact that the axis orrienations flip between link3 and the world frame so axis flips are also adjusted in the process to produce a roation matix that only tells us how the griper is rotated relative to link3 of the arm.

My code for this is here:

```
R3_w = T3.transpose()[:3,:3] * R_rpy
R3_6 = R3_w * (self.rot_z(np.pi/2) * self.rot_x(np.pi/2)).T
```

Wrist rotation values for theta4, theta5, and theta6 are computed using the tf.transformaions routine directly from the rotation matrix created.

```
a, b, g = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), axes='ryzy')
```

and:

```
theta4 = a
theta5 = b
theta6 = g
```

#### Wrist Flips and Alternate Arm Positions

For any given IK problem defined by gripper locaton and pose, there are many different arm configurations that can move the gripper to the correct location and pose.  The base can be rotated towards, or away from the wrist center. and links 2 and 3, can form a triangle with the elbow riased, or lowered -- creating 4 possible configurations for theta1, theta2, and theta3 to position the arm at the correct wirst center.

The test exercise will sometimes produce paths that riase the arm up and over the base to make it reach backwards and then lower it again.  Without adjusting for alternate base configurations this path will force the base to do a 180 deg flip in the middle of the motion which is slow, time consuming, and forces a momentary wild off-path swing of the gripper as the base flips around.  I wrote code to consider the alternate base configuration where joint2 can be allowed to be on the far side of the base away from the wrist center and it worked fine to stop the inital base flip motion.  However, the paths generated often then caused the based to twist around and force the arm to reach backwards to the shelf -- which was not possible beuase the joints in the arms could not bend far ebnnough to reach the shelf backwards like that.  So the base rotate was required anyway.  The more advanced code only changed where the base rotated, and was not able to produce paths without base roations due to the types of paths generated in this exercise.  So I didabled the base flip code in the final version, though the code is still in the source file.

The same alternate arm configuation problem of the base, exists for the wrist as well. But it's much worse for the wrist due to the large rotation range of joint 4 and joint 6 (+- 350 degs).  There are often as many as 6 or 8 different wrist configurations that can put the gripper into the correct orrientation.

This exercise generates paths for the gripper that are full of gripper rotations, and without care in picking the correct wrist configuration at each step, the gripper will often have to pause and do a full 360 flip to get from one point in the move to the next.

I wrote code to test all possible wrist configuations and pick the best one based on minimizing rotations from the previous orrientation.  Unlike the base flip code that didn't help improve the base flip problem in the paths of this exercise, the wrist orriention code greatly reduced the need for wrist flips in the paths.  They almost never happen now for the paths of this exercie. The gripper is almost always able to move through the motions without need for wrist flips in the middle of the motion.  At the end of the pick and place, when the arm moves back to the start, it now has to "unwind" twists created by the pick and place motions. So you will notice a set of smooth roations and flips to get the arm back to the zero starting position as it moves back from the bin, to the starting location.

To work, the code needs to know the previous configuration of the arm. From step to step in a move, this is just saved and used for the next stop.  But since the pick and drop moves are separate IK requests, I had to hack in a solution for this exercise to pass the last position of the pick, to the first move in the drop cycle, so that there were no extra wrist flips at the start of the drop move.

The hack above represents a protocol weakness of the IK request. It could impoved by including information about the starting configuation of the arm but doesn't now, so the hack above was needed to create optimal IK solutions that didn't add an extra unneeded wrist flip at the start of the drop cycle.

### 3. Project Implimentation

#### 3-1. Fill in the `IK_server.py`

My file can be found in my repository here: [IK_server.py](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/IK_server.py)

As submited the code is configured to run as an IK server, but it includes a good bit of addition test and and debug features that are either commented out or turned off.

The Kinematics code for the arm was all moved into a class called Kuka_KR210.  The early version of the code used sympy as per the examples in the lessons, but I got tired of debuging code that was running so slow so I rewrote all the math to use numpy and that made it run about 300 times faster.  The sympy version of the transformation code is still to be found in the `getT_sympy()` method which was also used above to produce the symbolic versions of the Transform tables for this write-up.  But the bulk of the rest of the code is all based on numpy.

The `do_FK()` method computes forward kinematics by taking a set of 6 joint angles and computing the end effector x, y, and z location and its pose in the form of row, pitch, and yaw values.  This uses the DH transformation matrices created by the `getT()` method.

The `do_IK()` method is the Inverse Kinematcis code in numpy which works as described in the previous section of this report. It makes use of some of the FK transformation code both in it's calucations and to verify the results are valid. If the IK every fails to find a valid answer the error is logged to stdout.

Computed arm angles are always logged to the server log.

The wrist configuration optimization code can be found in the `find_best_wrist()` method.

#### Results

My code seems to work well.  The test environment however is a bit flakly and unstable.  For example, it can at times, become misaligned so that valid joint positions will no longer move the arm to the correct shelf locations.  I have not spent the time to track down what the cause of this is, but it doesn't seem to be in my code.

The gripper at times, fails to grab the cylinder, even though it's correlctly positioned.  Again, seems to be an environment issue.  I added the delay code as talked about in the slack channel and that greatly improves the odds of the gripper grabbing the cylinder but doesn't totally fix the issue.

Likewise, the motion planer will create invalid paths at times.  It will sometimes create paths that cause the blue cylinder being held by the arm to hit the shelf as it swings.  This type of collision of the arm with the shelf might be the cause of the system losing alignment mentioned above.  It will also generate paths at times that the arm can't reach -- so it's generating paths out of the motion space of the arm.  I've only seen this happen when it makes a wild swing almost straight up above the base -- and when it makes the error, it's only a very small amount out of reach -- a cm or less I belive.  My code catches this and reports it on a message to stdout and moves the arm as close to the correct location as it can. The resulting motion is fine, just not stricly matching what was requested. It only seems to run into this problem for one or two postions of the move when I've seen it.  It's rare.

#### End Effector Location Error Plot

This is optional work suggested in the lesson. This data includes about 2 1/2 pick and place cycles because it collected data from multiple server requests until it had over 100 points to plot. The values are the distance from the starting px,py,pz location to the value computed by running IK, then FK, to re-compute the same point. The limit of 64 bit floating point representation is about e-16 so these values just represent the accumulation of floating point rounding errors in the IK and FK computations.
  
![Error Plot](https://github.com/curtwelch/RoboND-Kinematics-Project/blob/master/misc_images/error%20plot.png)

