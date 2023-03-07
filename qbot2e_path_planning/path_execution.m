%% Clear & Close all 
clear all
close all
clc

%% Load Path Execution
load path_execution.mat
[map_wth, map_len] = size(map); % size() returns number of row & column 
map_title = strcat("Map of ",string(map_len),"x",string(map_wth)," pixels");
figure('Name',map_title);       % create a new MATLAB figure
imshow(map);                    % display the map on the figure

%% Generate chain of command positions

for i = 1:length(goal_cells(:,1))
    % translate goal_cells w.r.t initial position
    goal_cells(i,:) = goal_cells(i,:) - init_robot_pos;
end
% translate goal_cells to physical position in (m)
goal_cells = goal_cells*pixel_size;
px = goal_cells(:,1);
py = goal_cells(:,2);



