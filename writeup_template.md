## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[DH_diagram]: ./writeup_images/DH_diagram.jpg
[DH_table]: ./writeup_images/DH_table.jpg
[FK]: ./writeup_images/FK_calculations.png
[Joint_angles]: ./writeup_images/theta1_2_3.jpg
[Dropoff]: ./writeup_images/dropoff.png
[Pickup]: ./writeup_images/pickup.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

After running the demo and evaluating the urdf file, axes were assigned to the robot as below:
![FK diagram][DH_diagram]
And here's the DH parameters table:s
![DH table][DH_table]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
Using the DH parameter table above, individual transforms are calculated as follows:
```
# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')   
# Create Modified DH parameters
s = {alpha0:      0, a0:      0, d1:  0.75, q1:          q1,
     alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2. + q2,
     alpha2:      0, a2:   1.25, d3:     0, q3:          q3,
     alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:          q4,
     alpha4:  pi/2., a4:      0, d5:     0, q5:          q5,
     alpha5: -pi/2., a5:      0, d6:     0, q6:          q6,
     alpha6:      0, a6:      0, d7: 0.303, q7:           0}            
# Define Modified DH Transformation matrix
def transform(alpha, a, d, q):
    T = Matrix([[           cos(q),           -sin(q),          0,              a],
          [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
          [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
          [                0,                 0,           0,             1]])
    return T
# Create individual transformation matrices
T0_1 = transform(alpha0, a0, d1, q1).subs(s)
T1_2 = transform(alpha1, a1, d2, q2).subs(s)
T2_3 = transform(alpha2, a2, d3, q3).subs(s)
T3_4 = transform(alpha3, a3, d4, q4).subs(s)
T4_5 = transform(alpha4, a4, d5, q5).subs(s)
T5_6 = transform(alpha5, a5, d6, q6).subs(s)
T6_G = transform(alpha6, a6, d7, q7).subs(s)

T0_2 = T0_1 * T1_2
T0_3 = T0_2 * T2_3
T0_4 = T0_3 * T3_4
T0_5 = T0_4 * T4_5
T0_6 = T0_5 * T5_6
T0_G = T0_6 * T6_G

# Correction rotation
R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
              [sin(pi), cos(pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
R_y = Matrix([[cos(-pi/2.), 0, sin(-pi/2.), 0],
              [0, 1, 0, 0],
              [-sin(-pi/2.), 0, cos(-pi/2.), 0],
              [0, 0, 0, 1]])
R_corr = R_z * R_y

T0_G = T0_G.subs(angles) * R_corr
```
Also a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose is calculated as follows:
```
R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(q1), -sin(q1)],
              [ 0,        sin(q1),  cos(q1)]])

R_y = Matrix([[ cos(q2),        0,  sin(q2)],
              [       0,        1,        0],
              [-sin(q2),        0,  cos(q2)]])

R_z = Matrix([[ cos(q3), -sin(q3),        0],
              [ sin(q3),  cos(q3),        0],
              [ 0,              0,        1]])
Rot_G = R_z * R_y * R_x
R_correction = R_z.subs(q3, pi) * R_y.subs(q2, -pi/2.)
Rot_G = Rot_G * R_correction
Rot_G = Rot_G.subs({q3: yaw, q2: pitch, q1: roll})
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
The first step is to calculate the wrest center using the homogeneous transform mentioned above in the FK section as follows:
```
Rot_G = Rot_G.subs({q3: yaw, q2: pitch, q1: roll})
EE = Matrix([[px],
             [py],
             [pz]])
WC = EE - 0.303 * Rot_G[:,2]
```
Then using the WC and the DH parameter table, the first three joint angles can be calculated as follows:

![First 3 angles][Joint_angles]

After that the second 3 joint angles are calculated using the following calculations:
```
R0_3 = T0_3[0:3,0:3].evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.inv("LU") * Rot_G

# R3_6 can be calculated from: T3_6 = T3_4*T4_5*T5_6
# solving both sides:
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(pow(R3_6[0,2],2)+pow(R3_6[2,2],2)),R3_6[1,2])
```
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Applying all the calculations explained above, I implemented them in `IK_server.py` file to help the robot pick and place objects successfully.

Although writing the equations only should guide the robot, but the arm movements were so slow, that it was impossible to complete the cycle, so I had to do some modifications to improve the performance as follows:
1- Some calculations were not dependant on the robot's current position, like for example the individual transform matrices, so I moved all those calculations outside the loop, this way, they were calculated only once.
2- Matrices multiplications are much faster with less unknown symbols, so I made sure to substitute symbols values before multiplications whenever possible.
3- `Simplify` function is very slow, and not needed, so I made sure not to use it in my script.

If I were going to pursue this project further, I'd work on improving the performace, and decreasing the unnecessary rotations performed by the last 3 joints while moving to the target or to the drop off locations.

Here are some screenshots showing the arm in action:

![pick up][Pickup]

![drop off][Dropoff]



