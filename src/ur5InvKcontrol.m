function [result, error] = ur5InvKcontrol(q_start, q_goal, ur5, steps)
%% Outputs 
% result : will return 1 if control is executed sucessfully
% error : final position error at end of control loop
%% Inputs
% q_start : Stating joint configuration
% Q_goal : Target goal configuration
% ur5 : ur5_interface() object
% steps : number of steps to take between start and goal

    %Function Hyper Parameters
    time = 1;
    
    %Obtain start and goal SE3
    g_goal = ur5FwdKin(q_goal);
    g_start = ur5FwdKin(q_start);
    
    %Interpolate in cart space
    points = interp(g_start, g_goal, steps);
    
    %Move to starting configuration
    disp("ur5InvKcontrl : Moving to start")
    ur5.move_joints(q_start, 5);
    pause(5);

    %Cal and display error in starting position
    q_start_m = ur5.get_current_joints();
    g_start_m = ur5FwdKin(q_start_m);

    gtt_start = g_start\g_start_m;
    xik_start = getXi(gtt_start);
    error_start = norm(xik_start(1:3));

    error_rot_start = sqrt(sum(diag((g_start_m(1:3,1:3) - g_start(1:3,1:3))*(g_start_m(1:3,1:3) - g_goal(1:3,1:3))')));

    disp("Start Error Rot ");
    disp(error_rot_start);

    disp("Start Error Pos ");
    disp(error_start);

    
    %Enter Control Loop
    disp("ur5InvKcontrol : Entering Control Loop")
    for i = 1:steps
        
        %Obtain ith joint config
        q = ur5InvKin_wrap(points(:,:,i));
        
        %Choose Optimal joint config
        [result, q_op, ind] = optimalJointConfig(ur5, q);
        
        
        %Check new pose for singularity
        Jb = ur5BodyJacobian(q_op);
        if(manipulability(Jb, 'invcond') < 0.01)
            disp(q_op);
            error = -1;
            return
        end

        %Move to next position
        if (result == 1)
            ur5.move_joints(q_op, time);
        else 
            disp("No optimal Q found");
            error = -1;
            return
        end
        
        pause(time);

    end
    
    %Calculate and display final error
    gst = ur5FwdKin(q_op);
    gtt = g_goal\gst;
    xik = getXi(gtt);
    error = norm(xik(1:3));

    result = 1;

    error_rot = sqrt(sum(diag((gst(1:3,1:3) - g_goal(1:3,1:3))*((gst(1:3,1:3) - g_goal(1:3,1:3))'))));

    disp("Target Error Rot ");
    disp(error_rot);

    disp("Target Error Pos ");
    disp(error);
    
    
end


