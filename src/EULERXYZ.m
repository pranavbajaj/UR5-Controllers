function R = EULERXYZ(angles)
if(angles(2)>pi/2 || angles(2)<-pi/2)
    error('The rotation along y axis should be in range [-pi/2,pi/2]');
else
R = ROTX(angles(1))*ROTY(angles(2))*ROTZ(angles(3));
end
end

