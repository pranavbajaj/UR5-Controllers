%Used for testing controllers on UR5 in Wyamn 170. INV, RR, JT flags can be
%changed to change controller. ur5_project.m is a better workflow with the
%same functionality

clc; 
clear;
steps = 10;

ur5 = ur5_interface(); 

ur5.get_current_joints()

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
    start_q = ur5.get_current_joints();
    start_location = ur5FwdKin(start_q); 
    disp('Start location recorded')
    disp(start_q);
end

disp('Move the robot to target location and then press enter')
w = waitforbuttonpress; 
if w == 0
    target_q = ur5.get_current_joints();
    target_location = ur5FwdKin(target_q); 
    disp('Target location recorded')
    disp(target_q);
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

[result, q_goal_1, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(target_location1));
[result, q_goal_2, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(target_location2));
q_goal_3 = target_q;

INV = true;
RR = false;
JT = false;
%---------------- Traj A using InvKin ----------------
% 
if INV
    disp("Entering Control Loop for InvKin Control : Traj A")

    disp("Drawing Line Segment 1");
    [result1, error1] = ur5InvKcontrol(q_start_1, q_goal_1, ur5, steps);

<<<<<<< Updated upstream
=======
    while( result1 ~= 1)
        result1 
        ur5.get_current_joints()

    end

>>>>>>> Stashed changes
    disp("Drawing Line Segement 2");
    [result2, error2] = ur5InvKcontrol(q_start_2, q_goal_2, ur5, steps);

    disp("Drawing Line Segement 3");
<<<<<<< Updated upstream
    [result3, error3] = ur5InvKcontrol(q_start_3, q_goal_3, ur5, steps);
=======
    
    while( result3 ~= 1)
>>>>>>> Stashed changes

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

    ur5RRcontrol(q_start_1, q_goal_1, ur5, 1)
    
    
    disp("Drawing Line Segement 2");

    ur5RRcontrol(q_start_2, q_goal_2, ur5, 1)
    
    disp("Drawing Line Segement 3");

    ur5RRcontrol(q_start_3, q_goal_3, ur5, 1)

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

    ur5JTcontrol(q_start_1, q_goal_3, ur5, 1)
        
    pause(2)
    disp('Moving back to home position')
    ur5.move_joints([0;-1.57;0;-1.57;0;0],5);
    pause(5)
    disp('At home location')
    
end

