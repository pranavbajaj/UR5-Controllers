clc; 
clear; 

% Initializing the ur5 interface object.
ur5 = ur5_interface(); 

% Getting the controllering type from user. 
disp("1. RR-based: Resolve Rate Controller")
disp("2. IK-Based: Inverse Kinematic Controller")
disp("3. TJ-Based-: Jacobian Transpose Controller")
prompt = "Enter the Controller type: ";
c = input(prompt,"s");

% Home Configuration. 
home_cfg = [0;-1.57;0;-1.57;0;0];
steps = 10; 

% Moving robot to the home configuration.
if abs((norm(ur5.get_current_joints()) - norm(home_cfg))) <= 1e-3
    disp('UR5 is already at the home configuration')
else 
    disp('Moving to home loaction...')
    ur5.move_joints(home_cfg,5)
    pause(5)
    disp('At home location')
end

% Recording the start location. 
disp('Moved the robot to start location and then click on the prompted window')
w = waitforbuttonpress;
if w == 0
    start_q = ur5.get_current_joints();
    start_location = ur5FwdKin(start_q); 
    disp('Start location recorded')
    disp(start_q);
end

% Recording the target location. 
disp('Move the robot to target location and then click on the prompted window')
w = waitforbuttonpress; 
if w == 0
    target_q = ur5.get_current_joints();
    target_location = ur5FwdKin(target_q); 
    disp('Target location recorded')
    disp(target_q);
end

% Moving the robot back to the home configuration. 
disp("Moving back to home position")
ur5.move_joints(home_cfg,5)
pause(5)
disp('At home location')

% Internal code to record the intermediate points. 
s = [start_location(1,4), start_location(2,4)];
t = [target_location(1,4), target_location(2,4)];
points = intermediatePoints(s,t);
start_location1 = start_location;
target_location1 = start_location;
target_location1(1,4) = points(1,3); 
target_location1(2,4) = points(1,4); 
start_location2 = target_location1; 
target_location2 = start_location;
target_location2(1,4) = points(2,3); 
target_location2(2,4) = points(2,4); 
target_location2(3,4) = target_location(3,4);
start_location3 = target_location2;
target_location3 = start_location;
target_location3(1,4) = points(3,3); 
target_location3(2,4) = points(3,4); 
q_start_1 = start_q;
[result, q_start_2, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(start_location2));
[result, q_start_3, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(start_location3));
[result, q_goal_1, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(target_location1));
[result, q_goal_2, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(target_location2));
q_goal_3 = target_q;


% RR Control Loop
if (c == "RR-based")
       
    disp("Entering Control Loop for RR Control : Traj A")
    disp("Drawing Line Segment 1");
    error1 = ur5RRcontrol(q_start_1, q_goal_1, ur5, 1);
    
    disp("Drawing Line Segement 2");
    error2 = ur5RRcontrol(q_start_2, q_goal_2, ur5, 1);
    
    disp("Drawing Line Segement 3");
    error3 = ur5RRcontrol(q_start_3, q_goal_3, ur5, 1);
    pause(2)

    disp('Moving back to home position')
    ur5.move_joints(home_cfg,5);
    pause(5)
    disp('At home location')


% InvK Control Loop
elseif (c == "IK-based")

    disp("Entering Control Loop for InvKin Control : Traj A")
    disp("Drawing Line Segment 1");
    [result1, error1] = ur5InvKcontrol(q_start_1, q_goal_1, ur5, steps);
    

    disp("Drawing Line Segement 2");
    [result2, error2] = ur5InvKcontrol(q_start_2, q_goal_2, ur5, steps);
    

    disp("Drawing Line Segement 3");
    [result3, error3] = ur5InvKcontrol(q_start_3, q_goal_3, ur5, steps);
    pause(2)

    disp('Moving back to home position')
    ur5.move_joints(home_cfg,5);
    pause(5)
    disp('At home location')


% JT control Loop. 
elseif (c == "TJ-based")
    
    disp("Entering Control Loop for JT Control : Traj B")

    disp("Drawing Line Segment 1");
    error1 = ur5JTcontrol(q_start_1, q_goal_3, ur5, 1);
    pause(2)
    
    disp('Moving back to home position')
    ur5.move_joints(home_cfg,5);
    pause(5)
    disp('At home location')

else
    disp("Invalid Controller")
end