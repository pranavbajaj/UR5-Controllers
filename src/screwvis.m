ur5=ur5_interface();
pause(0.3);
Frame_M = tf_frame('base_link', 'Frame_M', eye(4));

xi = [1 ;0 ;1/(2*pi) ;0; 0 ;1];
v = xi(1:3);
w = xi(4:6);
xi_hat = [SKEW3(w), v;...
    0 0 0 0];
frame = 200;
interval = 2*pi/frame;

for i = 0:frame
    theta = i*interval;
    H = expm(xi_hat*theta);
    Frame_M.move_frame('base_link', H);
    pause(0.3);
end
