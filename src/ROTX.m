function RotationX = ROTX(q)
%ROTX Summary of this function goes here
%   input: radians
    RotationX = [1,0,0;...
        0,cos(q),-sin(q);...
        0,sin(q),cos(q)];
end

