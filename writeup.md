## Project: Kinematics Pick & Place
Student writeup by Curt Welch <curt@kcwc.com>
July 7, 2017
Project submited as a github repository here: https://github.com/curtwelch/RoboND-Kinematics-Project

---

[//]: # (Image References)

[dh-drawing]: ./misc_images/DH Drawing.jpg
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

![alt text][image1]

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup 

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![DH Drawing][dh-drawing]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py`


The file can be found in my repository here:

  


