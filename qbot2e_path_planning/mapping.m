%{
* Notation:
    x <=> (matrix) column <=> horizontal direction <=> length (of figure)
    y <=> (matrix) row    <=> verical direction    <=> width (of figure)
    w.r.t = with respect to
%}

%% Clear & Close all 
clear all
close all
clc

%% Pamameter
pixel_size = 0.1;       % (m), physical length of a pixel
unknown_val = 100;      % Value of unknown pixel
obs_val = 0;            % Vale of occupied pixel
free_val = 255;         % Value of free pixel

%% Load Map
load map_obs.mat

% ----- RESIZE MAP ----- %

in = input('Resize the map? Full map takes longer to process algorithms. Type 1 = yes, 0 = no ');
if in == 1
   % vertical resize values (y-direction)
%    wth_start = 95;         
%    wth_end = 210;
   wth_start = 155;         
   wth_end = 180;
   % horizontal resize values (x-direction)
%    len_start = 75;
%    len_end = 200;
   len_start = 130;
   len_end = 165;
   % Concatenate a string to display on command window
   text = strcat("Default values for map resize: ", ...
   string(wth_start),':',string(wth_end),',',string(len_start),':',string(len_end), ...
   " (e.g. for width, it take from pixel ",string(wth_start)," to pixel ",string(wth_end),')');
   disp(text);
   disp("Adjust resize values directly in the MATLAB script");
   % Assign resize values for map
   map = map_obs(wth_start:wth_end,len_start:len_end);
else
    % Use full map
   map = map_obs; 
   disp('skipped map resize');
end

% ----- DISPLAY RESIZED MAP ----- %
[map_wth, map_len] = size(map); % size() returns number of row & column 
map_title = strcat("Map of ",string(map_len),"x",string(map_wth)," pixels");
figure('Name',map_title);       % create a new MATLAB figure
imshow(map);                    % display the map on the figure


% Note: in "map" variable, value of grey is 100, black is 255, white is 0
% Grey area if unknown area, black is occupied, white is free

text = strcat("Size of one pixel is ",string(pixel_size),'x',string(pixel_size),' (m)', ...
". Adjust pixel size directly in the MATLAB script");
disp(text);


%% Map Modification

% ----- REINSERT BOX ON MAP ----- %
in = input("Reinsert the box on map? Type 1 for yes, 0 for no ");
if in == 1
    
    % Define size of the box
    box_size = [0.5, 0.5];  % size of the box in (m)
    safe_region = 0.3;      % to avoid collision with robot
    box_size = box_size + safe_region;
    box_pixel = box_size/pixel_size;    % Translate box_size to pixels
    
    % Ask for user input on the graph
    text = strcat("Click on the map to insert box size of ", ...
    string(box_size(1)),'x',string(box_size(2)),' (m)', ...
    ". Adjust box size directly in the MATLAB script");
    disp(text);
    
    % Take user input
    box_pos = ginput(1); % Position of the box on map in pixel, with decimals
    % note that ginput() returns x-y (col-row) instead of row-col (y-x)
    box_pos = round(box_pos);   % Round to omit decimal
    
    % Find box center w.r.t box size
    box_cen = round(box_pixel/2);
 
    % Distance from two extremes to box center (w.r.t box size)
    box_dist = [box_pixel(1)-box_cen(1) box_pixel(2)-box_cen(2)];

    % Go through entire map & fill the box on map
    for i = (box_pos(2) - box_dist(2)):(box_pos(2) + box_dist(2))
        for j = (box_pos(1) - box_dist(1)):(box_pos(1) + box_dist(1))
            map(i,j) = obs_val;
        end
    end
    
    % Display map again
    close all
    figure('Name',map_title);   % create a new MATLAB figure
    imshow(map);                % display the map on the figure
    
else
    disp('Skipped box reinsert');
end

% ----- CLEAR NOISE ----- %
in = input("Clear noise? Type 1 for yes, 0 for no. ");
if in == 1
    in2 = input("Manual or auto? Type 1 for auto, 0 for manual ");
    
    if in2 == 1

        clear_dist = 2; % Number of pixel to check from an occupied pixel         
        % Go through each pixel of entire map
        for i = (clear_dist+1):(map_wth-clear_dist) % should pad the image
            for j = (clear_dist+1):(map_len-clear_dist)
                
                % If the pixel is occupied, then ...
                if map(i,j)==obs_val
                    % ... check the vicinity of the pixel
                    not_free_count = 0; % count the number of occupied pixels
                    for h = (i-clear_dist):(i+clear_dist)
                        for k = (j-clear_dist):(j+clear_dist)
                            if (h ~= i)&&(k ~= i)&&((map(h,k)==obs_val)||(map(h,k)==unknown_val))
                                not_free_count = not_free_count + 1;
                            end
                        end
                    end
                    % if number of occupied pixel in the vicinity is less
                    % than a certain number, it is likely to be noise
                    if not_free_count < 2
                        map(i,j)= free_val;
                    end
                end    

            end
        end
        disp('Automatically clear noise');
    else
        disp("Click on pixels user want to delete. Press Enter (return) on the figure to end");
        del_pixel = ginput;
        del_pixel = round(del_pixel);
        
        % Clear the map
        clear_dist = 2; % define how many pixel from mouse click to delete
        
        % Go through the entire list of picked pixels to clear
        for i = 1:length(del_pixel(:,1))
            
            % Clear area around the picked pixels
            for h = (del_pixel(i,2)-clear_dist):(del_pixel(i,2)+clear_dist)
                for k = (del_pixel(i,1)-clear_dist):(del_pixel(i,1)+clear_dist)
                    map(h,k) = free_val;
                end
            end
        end
    end
    
    % Display map again
    close all
    figure('Name',map_title);   % create a new MATLAB figure
    imshow(map);                % display the map on the figure
else
    disp('Skipped clearing noise');
end

%% Extract robot position and orientation
% note that ginput() returns x-y (col-row) instead of row-col (y-x)
disp('Click to record robot initial position on map');
init_robot_pos = ginput(1);
init_robot_pos = round(init_robot_pos);
disp('Click to record robot initial orientation on map');
init_robot_orien = ginput(1);
init_robot_orien = round(init_robot_orien);

%% Save the MATLAB workspace
save('map_extracted.mat')
disp('Extracted the MATLAB workspace. This file can be closed now');