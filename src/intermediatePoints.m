function points = intermediatePoints(s,t)
% s: start location (x,y)
% t: target location (x,y)
int_x = s(1) + (t(1) - s(1)) / 2; 
e1 = [s(1), s(2), int_x, s(2)]; 
e2 = [int_x, s(2), int_x, t(2)];
e3 = [int_x, t(2), t(1), t(2)]; 

points = [e1;e2;e3]; 
end