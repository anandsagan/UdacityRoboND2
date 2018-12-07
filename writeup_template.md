## Project 2: Kinematics Pick & Place (KUKA KR210)
### Author: Anand Kannan
---


**Steps taken to complete the project:**


1. Set up ROS Workspace.
2. Cloned the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of  ROS Workspace.
3. Experimented with the forward_kinematics environment and familiarized myself with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

---
### Writeup
In order to successfully complete this project, I used the following environment:
- Ubuntu 16.04
- ROS Kinetic
- Gazebo 7
- RVIZ 1.12.16


### Kinematic Analysis
#### Running Forward Kinematics Demo
I ran the forward kinematics demo as shown below to familarize myself with the RVIZ environment, and to see how each joint reacts when the joint space parameters were adjusted and the direction and orientation of each joint in the cartesian space. To run the demo, I ran the following shell script: 

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

**Modified DH Parameter Table**

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | ---     | ---    | ---  | ---
0-1 | 0       | 0      | 0.75 | qi
1-2 | - pi/2  | 0.35   | 0    | - pi/2 + q2
2-3 | 0       | 1.25   | 0    | q3
3-4 |  - pi/2 | -0.054 | 1.50 | q4
4-5 | pi/2    | 0      | 0    | q5
5-6 | - pi/2  | 0      | 0    | q6
6-EE| 0       | 0      | 0.303| 0

The general transformation matrix between two frames is shown below:
**General Transformation Matrix**
cos(q)|-sin(q)|0|a
sin(q)*cos(alpha) | cos(alpha)|-sin(alpha)|-sin(alpha)*d
sin(q)*sin(alpha)|cos(q)*sin(alpha)|cos(alpha)|-cos(alpha)*d
0|0|0|1
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.


And just for fun, another example image:
![alt text][image3]


