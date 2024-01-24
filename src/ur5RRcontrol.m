function error = ur5RRcontrol(q_start, q_goal, ur5, K)
%% Outputs 
% error : final position error at end of control loop
%% Inputs
% q_start : Stating joint configuration
% Q_goal : Target goal configuration
% ur5 : ur5_interface() object
% K : Intial gain value

    %Function Hyper Parameters
    posThresh = .005;
    rotThresh = .02618;
    T = 0.01;
    
    %Move to start configuration
    disp("ur5RRcontrl : Moving to start configuration")
    ur5.move_joints(q_start, 5);
    pause(5);
    
    %Obtain Goal Pose and Initial Error
    g_goal = ur5FwdKin(q_goal);
    qk = ur5.get_current_joints();
    gst = ur5FwdKin(qk);
    gtt = g_goal\gst; 
    xik = getXi(gtt);
    

    %Cal and display error in starting position
    g_start = ur5FwdKin(q_start);
    q_start_m = ur5.get_current_joints();
    g_start_m = ur5FwdKin(q_start_m);

    gtt_start = g_start\g_start_m;
    xik_start = getXi(gtt_start);
    error_start = norm(xik_start(1:3));

    error_rot_start = sqrt(sum(diag((g_start_m(1:3,1:3) - g_start(1:3,1:3))*((g_start_m(1:3,1:3) - g_goal(1:3,1:3))'))));

    disp("Start Error Rot ");
    disp(error_rot_start);

    disp("Start Error Pos ");
    disp(error_start);
    
    %Resolved Rate Control Loop
    disp("ur5RRControl : Entering Control Loop")
    while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)

        Jb = ur5BodyJacobian(qk);

        %Dynamically determine K such that max joint velocity is a constant
        v = inv(Jb)*xik;
        qv = K*T*inv(Jb)*xik;
        [val, index] = max(abs(qv));
        K = (pi/2)/(abs(v(index))*T)/50;
        K = min(50, K);

        %Update Step
        qk_1 = qk-K*T*inv(Jb)*xik;
        qk = qk_1;
        
        %Obtain New Error
        gst = ur5FwdKin(qk);
        gtt = g_goal\gst;
        xik = getXi(gtt);
        
        %Check new pose for singularity
        if(manipulability(Jb, 'invcond') < 0.01)
            disp(qk);
            error = -1;
            return
        end

        %Move to new position
        ur5.move_joints(qk, 0.5);
        pause(0.5);
    end

    %Calculate and display Final error
    error = norm(xik(1:3));
    error_rot = sqrt(sum(diag((gst(1:3,1:3) - g_goal(1:3,1:3))*((gst(1:3,1:3) - g_goal(1:3,1:3))'))));
    disp("Target Error Rot ");
    disp(error_rot);

    disp("Target Error Pos ");
    disp(error);

end

    
