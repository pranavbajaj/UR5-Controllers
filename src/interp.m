function [points] = interp(g_start, g_final, steps)
%Given an SE3 this function will perform linear interpolation on the
%position vector. Returns a series of intermediate SE3 points which have
%same orientation as given start SE3.


    %Buffer for assigning points (Unneccesary)
    g_temp = g_start;

    %Initializing point array
    points = repmat(g_start, [1,1, steps]);
    p_start = g_start(1:3, 4);
    p_final = g_final(1:3, 4);

    %Populate Point array 
    for i = 1:steps
        g_temp(1:3, 4) = p_start + ((p_final-p_start)/steps)*i;
        points(:,:,i) = g_temp();

    end


end