function [ theta ] = ur5InvKin_wrap( gd )
%   Input args:
%           gd - the 4x4 matrix - desired transformation from base to end
%   Output args:
%           theta - 6x8 matrix, each colum represents one possible solution 
%           of joint angles
dummy = [0;0;0];
t = [0;0;89.2/1000];
gripper = [1 0 0 0;...
           0  1 0 -49/1000;...
           0 0 1 122.28/1000;...
           0 0 0 1];  
% transform the gst to the setiing to use the provided ur5InvKin
gd = [ROTZ(pi/2),-t;0 0 0 1]*gd*inv(gripper)*[ROTY(-pi/2),dummy;0 0 0 1]*[ROTX(pi/2),dummy;0 0 0 1];

theta_temp = ur5InvKin(gd);
%cope the zero configuration difference
theta_temp(2,:) = theta_temp(2,:) - pi/2;
theta_temp(4,:) = theta_temp(4,:) - pi/2;
theta = theta_temp;
end
