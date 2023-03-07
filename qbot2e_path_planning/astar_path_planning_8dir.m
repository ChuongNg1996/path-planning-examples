%{
* Notation:
    x <=> (matrix) column <=> horizontal direction <=> length (of figure)
    y <=> (matrix) row    <=> verical direction    <=> width (of figure)
    position <=> cell <=> pixel
    w.r.t = with respect to
%}

%% Clear & Close all 
clear all
close all
clc

%% Pamameter
pixel_size = 0.1;       % (m), physical length of a pixel
unknown_val = 100;      % Value of unknown pixel
obs_val = 0.2;            % Vale of occupied pixel
free_val = 255;         % Value of free pixel

% astar paramaters
obs_val_new = -1;       % new value for obstacle b/c at init position f(n)=0
direction_num = 8;      % Number of direction
large_num = 10000.0;    % Arbitrary large number for finding minimum algorithm
h_factor = 0.0;         % h factor for h(n)
used_cell = zeros(1,2); % an array to store used cells,
                        % first element is column, next element is row

%% Load Map
load map_extracted.mat
[map_wth, map_len] = size(map); % size() returns number of row & column 
map_title = strcat("Map of ",string(map_len),"x",string(map_wth)," pixels");
figure('Name',map_title);       % create a new MATLAB figure
imshow(map);                    % display the map on the figure

%% Path Planning Algorithm
% note that ginput() returns x-y (col-row) instead of row-col (y-x)
disp('Click to define desired position on map');
goal_pos = ginput(1);
goal_pos = round(goal_pos);

% Calculate f(n) map
f_map = double(map);

for i = 1:map_wth 
    for j = 1:map_len
        if f_map(i,j) ~= obs_val
            dist_g = sqrt((i-init_robot_pos(2))^2 + (j-init_robot_pos(1))^2);
            dist_h = sqrt((i-goal_pos(2))^2 + (j-goal_pos(1))^2);
            f_map(i,j) = dist_g + h_factor*dist_h;
        else
            % replace new value for obstacle b/c at init position f(n)=0
            f_map(i,j) = obs_val_new;
        end
    end
end

% Assign current cell as goal cell, then make a reverse path to initial cell
current_pos = goal_pos;

% While current cell is not initial cell ...
while (current_pos(2) ~= init_robot_pos(2)) || (current_pos(1) ~= init_robot_pos(1))
    min_f = large_num;
    
    % Scan the vicinity the current cell
    for i = 1:direction_num
        
        if i == 1
            row = current_pos(2) - 1;   % vertical up
            col = current_pos(1);       % horizontal same
        elseif i == 2
            row = current_pos(2) - 1;   % vertical up
            col = current_pos(1) - 1;   % horizontal left
        elseif i == 3
            row = current_pos(2);       % vertical same
            col = current_pos(1) - 1;   % horizontal left
        elseif i == 4
            row = current_pos(2) + 1;   % vertical down
            col = current_pos(1) - 1;   % horizontal left
        elseif i == 5
            row = current_pos(2) + 1;   % vertical down
            col = current_pos(1);       % horizontal same
        elseif i == 6
            row = current_pos(2) + 1;   % vertical down
            col = current_pos(1) + 1;   % horizontal right
        elseif i == 7
            row = current_pos(2);       % vertical same
            col = current_pos(1) + 1;   % horizontal right
        elseif i == 7
            row = current_pos(2) + 8;   % vertical up
            col = current_pos(1) + 1;   % horizontal right
        end
        
        % Check if row or col is out of the image
        limit_bool = 0;
        if (row < 1) || (row > map_wth) || (col < 1) || (col > map_len)
            limit_bool = 1;
        end
        
        if limit_bool == 0
            % Check if the cell is occupied or not
            if f_map(row,col) ~= obs_val_new

                % Check if the cell has been used, avoid looping
                used_bool = 0;
                for j = 1:length(used_cell(:,1))
                    if (col == used_cell(j,1)) &&  (row == used_cell(j,2))
                        used_bool = 1;
                    end
                end

                % if the cell is used, then skip it, else check if it has
                % smallest f(n) in vicinity of the current cell
                if used_bool == 0
                    % if the considered cell is smaller than current min
                    if (f_map(row,col) < min_f)
                        % replace current min in vicinity of current cell
                        min_f = f_map(row,col); 
                        
                        % replace position of next cell
                        next_row = row;
                        next_col = col;
                    end
                end
            end
        end
    end
    
    % After scanning all directions and found smallest f(n) cell, assign it
    % to current_pos and stored array
    current_pos(1) = next_col; 
    current_pos(2) = next_row;
    used_cell = [used_cell; next_col next_row];
end

%% Plot path
for i = 2:length(used_cell(:,1))
    map(used_cell(i,2),used_cell(i,1)) = 100;
end
close all
figure('Name',map_title);       % create a new MATLAB figure
imshow(map);                    % display the map on the figure

%% Save Path Implementation
goal_cells = used_cell(2:end,:);    % omit (0,0)
goal_cells = flip(goal_cells);      % flip the sub goals since the element starts from goal
save('path_execution.mat','init_robot_pos','init_robot_orien', ...
'pixel_size','goal_cells','map');
disp('Extracted path implementation. This file can be closed now');