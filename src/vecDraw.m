function result = vecDraw(q_start,pointVec,ur5)
%% Outputs 
% result : returns 1 if all points have been drawn
%% Inputs
% q_start : Stating joint configuration
% pointVec : points x 2 vector of x,y cordinates to be drawn. 1st point is
% expected to be 0,0
% ur5 : ur5_interface() object

    %Intialize data members
    time = 0.3;
    steps = size(pointVec(:,1));
    g_start = ur5FwdKin(q_start);
    points = repmat(g_start, [1,1,steps]);
    q_op = zeros([6, steps]);
    
    %Move to home before starting
    disp("Moving Home");
    ur5.move_joints([0; -1.57; 0 ; -1.57; 0 ; 0], 5);
    pause(5);

    %Move to start configuration   
    disp("VecDraw : Moving to start configuration");
    ur5.move_joints(q_start, 5);
    pause(5);
    disp("At Start Config");
    disp(q_start);

    %Iterate through pointVec and populate a 6 x points array of sequantial
    %joint configurations
    disp("Entering Drawing Loop");
    for i = 1:steps
<<<<<<< Updated upstream

        points(1:2, 4, i) = points(1:2, 4, i) + pointVec(i,:)';
        
=======
        
        points(1:2, 4, i) = points(1:2, 4, i) + pointVec(i,:)';

>>>>>>> Stashed changes
        q_i = ur5InvKin_wrap(points(:,:,i));
        [result, q_op_i, ind] = optimalJointConfig(ur5, q_i);
        q_op(:,i) = q_op_i;
        
                     
    end
    
    %Send joint confiuration array to controller and hold script
    ur5.move_joints(q_op, time);
    pause(time*steps);
    
    %Move home after completing drawing
    disp("Moving Home");
    ur5.move_joints([0; -1.57; 0 ; -1.57; 0 ; 0], 5);
    pause(5);

   result = 1;
   
end