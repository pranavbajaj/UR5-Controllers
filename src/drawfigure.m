%script to draw any figure.
%1. when empty figure window comes, move the robot to the starting point
%2. click the empty figure, then draw on the new figure
%3. when done drawing, press enter

clc;
clear;
global ur5
global skip
skip = 3;
ur5 = ur5_interface();
scale = 10;
global q_start;

w = waitforbuttonpress;
if w == 0
%     q_start = [1.57, 0, -1.57, 0, 1.57, 0.7]';
    q_start = ur5.get_current_joints();
    disp('Start location recorded')
    disp(q_start);
end

% Create a figure as a canvas
figure();
canvas = [1 1; 1 -1; -1 -1; -1 1]/scale;
hold on
plot(canvas(:, 1), canvas(:, 2), 'r-');
plot(0, 0, 'ro');
hold off;
% used to only record every three points 
global count;
count = 0;
% Initialize an empty array to store the points
global points;
points = [];
global mouse_down;
mouse_down = false;
% Set the callback functions for the plot
set(gcf, 'WindowButtonMotionFcn', @dragCallback);
set(gcf, 'WindowButtonUpFcn', @end_drag);
set(gcf, 'WindowButtonDownFcn', @start_drag);
set(gcf, 'KeyPressFcn', @check_key);
disp('Drag over the plot to record points:');
w = waitforbuttonpress;
disp("Number of Points");
disp(size(points(:,1)));


% Define a function to be called when the user drags the mouse over the plot
function dragCallback(src, evt)
    global mouse_down;
    global points
    global count;
    count = count +1;
    global skip
    
    if ((mouse_down == true) && (mod(count,skip) == 0))
    % Get the current point and add it to the array
    point = get(gca, 'CurrentPoint');
    disp(point(1,1:2));
    points = [points; point(1,1:2)];
    % Update the plot with the new point
    hold on
    plot(point(1,1), point(1,2), 'b*');
    hold off
    end
end
function start_drag(~, ~)
    global mouse_down;
    mouse_down = true;
end
% Define the function that clears the mouse-down flag
function end_drag(~, ~)
    global mouse_down;
    global points;
    mouse_down = false;
end

% Define the function that checks if the "Enter" key is pressed
function check_key(~, event)
    global points
    global q_start
    global ur5
    % Check if the "Enter" key is pressed
    if strcmp(event.Key, 'return')
        points(:,1) = points(:,1) - points(1,1);
        points(:,2) = points(:,2) - points(1,2);
        plot(points(:, 1), points(:, 2), 'b-');
        disp("Drawing");  
        vecDraw(q_start, points, ur5);
    end
    
end



