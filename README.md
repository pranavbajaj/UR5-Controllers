# Controllers for UR5

<img src="images/ur5_RVIZ.png" width="720" height="600" />

## Inverse Kinematic Controller 

* The inverse kinematics controller relies upon the analytical ur5 inverse kinematics solution.
Given a start and end location represented as an SE3, a series of intermediate points (SE3’s)
are generated via linear interpolation in the x, y, and z coordinates. The inverse kinematics solution
is applied to these points, generating a series of sequential joint configurations that trace a
linear path in cartesian space. To select from the 8 potential joint configuration solutions for a
given SE3 we choose the joint configuration which is “closest” to the robot's current pose.
Closest means the L2 norm between a given joint configuration and the robot's current pose is
minimal. Executing this final array of joint configurations using the ur5 controller produces the
desired behavior


## Resolved Rate Controller 

The resolved rate controller relies upon the resolved rate equation, specified below. 
Given a start and end configuration, we enter a control loop that will execute until the current
rotation and position error are below a finite threshold. For each iteration of the control loop a
body jacobian, error twist, and gain value K are calculated. K is determined dynamically such
that the maximum joint velocity of the ur5 is held constant at any given time. Using these values
a “step” is taken and a new joint configuration is calculated. The manipulability of this new joint
configuration is measured using the inverse condition number of the jacobian to ensure it is not
singular or nearly singular. Given the manipulability of the proposed joint configuration is above
a given threshold it is sent to the ur5 controller and a new error value is calculated.

<img src="images/rr.ong" width="400" height="50" />

## Jacobian Transpose Controller

The Jacobian Transpose controller relies upon the JT control update equation, specified below.
After replacing the update equation the control loop for Jacobian transpose control is identical to
that of the resolved rate control with the exception of the use of a smaller time step.

<img src="images/jt.ong" width="400" height="50" />


# Results 

* All error terms were measured in UR5 simulation.

<img src="images/jt.ong" width="720" height="400" />
