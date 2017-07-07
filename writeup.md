## Project: Kinematics Pick & Place
### Student Writeup by Curt Welch <curt@kcwc.com>
### July 7, 2017
### Project submited as a github repository here: https://github.com/curtwelch/RoboND-Kinematics-Project

---

[//]: # (Image References)

[dh-drawing]: ./misc_images/DH Drawing.jpg
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

![alt text][image1]

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Metrics

#### 1. Provide a Writeup 

You're reading it!

#### 2. Kinematic Analysis
##### 2-1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

A hand-drawn diagram was suggested, so here's the hand-drawn notes I created for the project!

![DH Drawing](./misc_images/DH Drawing.jpg)

The Modified DH Table from my code:

Link Row | Alpha | a | d | q
--- | --- | --- | --- | ---
1 | 0 | 0 | 0 | theta1
2 | -pi/2 | 0.35 | 0.75 | theta2 - pi/2
3 | 0| 1.25 | 0 | theta3
4 | -pi/2 | -0.054 | 0 | 


s = {alpha0: 0,      a0: 0,
            alpha1: -pi/2,  a1: 0.35,   d1: 0.75,
            alpha2: 0,      a2: 1.25,   d2: 0,
            alpha3: -pi/2,  a3: -0.054, d3: 0,
            alpha4: pi/2,   a4: 0,      d4: 1.5,
            alpha5: -pi/2,  a5: 0,      d5: 0,
            alpha6: 0,      a6: 0,      d6: 0,
                                        d7: 0.303,  q7: 0}

##### 2-2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

##### 2-3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py`


The file can be found in my repository here: (kuka_arm/scripts/IK_server.py)
  



