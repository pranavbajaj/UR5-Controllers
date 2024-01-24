%Used for testing contorl algoirthms in simulation. Change flags INV, RR,
%JT to select a given control alorithm. Goal and Start are hard coded for
%quick testing

clc; 
clear;
steps = 10;

ur5 = ur5_interface(); 



if abs((norm(ur5.get_current_joints()) - norm([0;-1.57;0;-1.57;0;0]))) <= 1e-3
    disp('UR5 is already at the home configuration')
else 
    disp('Moving to home loaction...')
    ur5.move_joints([0;-1.57;0;-1.57;0;0],5)
    pause(5)
    disp('At home location')

end


disp('Moved the robot to start loaction and then press enter')

w = waitforbuttonpress;
if w == 0
    start_q = [-0.63, -0.94, 1.76, -2.14, -1.63, -0.19]';
    start_location = ur5FwdKin(start_q); 
    disp('Start location recorded')
end

disp('Move the robot to target location and then press enter')
w = waitforbuttonpress; 
if w == 0
    target_q = [1.26, -0.94, 1.76, -2.14, -1.63, -0.19]';
    target_location = ur5FwdKin(target_q); 
    disp('Target location recorded')
end

disp('Moving back to home position')
ur5.move_joints([0;-1.57;0;-1.57;0;0],5);
pause(5)
disp('At home location')

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

[~, q_goal_1, ind] = optimalJointConfig(ur5,ur5InvKin_wrap(target_location1));
[result, q_goal_2, ind] = optimalJointConfig(ur5,ur5InvKin_wrap(target_location2));
q_goal_3 = target_q;
%Source of large rot error


INV = false;
RR = false;
JT = true;

%---------------- Traj A using InvKin ----------------
% 
if INV
    disp("Entering Control Loop for InvKin Control : Traj A")

    disp("Drawing Line Segment 1");
    [result1, error1] = ur5InvKcontrol(q_start_1, q_goal_1, ur5, steps);


    disp("Drawing Line Segement 2");
    [result2, error2] = ur5InvKcontrol(q_start_2, q_goal_2, ur5, steps);

    disp("Drawing Line Segement 3");
    [result3, error3] = ur5InvKcontrol(q_start_3, q_goal_3, ur5, steps);

    pause(2)
    disp('Moving back to home position')
    ur5.move_joints([0;-1.57;0;-1.57;0;0],5);
    pause(5)
    disp('At home location')
    
end


%----------------Traj A using RR control---------------

if RR
    
     disp("Entering Control Loop for RR Control : Traj A")
    
    disp("Drawing Line Segment 1");
    
    while(ur5RRcontrol(q_start_1, q_goal_1, ur5, 1)  > 0.005)
    
        disp("Moving")
    
    end
    
    disp("Drawing Line Segement 2");
    
    while(ur5RRcontrol(q_start_2, q_goal_2, ur5, 1)  > 0.005)
    
        disp("Moving")
    end
    
    disp("Drawing Line Segement 3");
    
    while(ur5RRcontrol(q_start_3, q_goal_3, ur5, 1)  > 0.005)
    
        disp("Moving")
    
    end
    
    pause(2)
    disp('Moving back to home position')
    ur5.move_joints([0;-1.57;0;-1.57;0;0],5);
    pause(5)
    disp('At home location')
    
end


%---------------Generate traj B --------------- using start_location 1 =
%target Location 1

if(JT)
 disp("Entering Control Loop for JT Control : Traj B")
    
    disp("Drawing Line Segment 1");

    ur5JTcontrol(q_start_1, q_goal_3, ur5, 1);
    
    pause(2);
    disp('Moving back to home position')
    ur5.move_joints([0;-1.57;0;-1.57;0;0],5);
    pause(5);
    disp('At home location')
    
end

