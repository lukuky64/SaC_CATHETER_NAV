% storing clicked points in an array
click_coordinates = [];

%% Each team member should label a difference set of images
starting_image = 0;
set_ = 'a';
member_name = input("Enter your name: ", "s");
member_name = char(member_name);

switch member_name
    case char("lucas")
        disp("showing set a");
        set_ = 'a';
        starting_image = 0;
    case char("jack")
        disp("showing set b");
        set_ = 'b';
        starting_image = 1;
    case char("pat")
        disp("showing set c");
        set_ = 'c';
        starting_image = 2;
    case char("lucas2")
        disp("showing set d");
        set_ = 'd';
        starting_image = 5;
end

% Get the image dataset folder path
% folder_location_ = 'C:\Users\priva\OneDrive\Desktop\Aortic_catheter_project\Data2_Soft_pullback_1\Images\';
folder_location_ = input("Enter the location of the image folder: ", "s");
folder_location_ = char(folder_location_);

%% looping through selected set of images for labelling
for i = starting_image:10:2000
    image_label = [num2str(i), '.jpg'];
    imagePath = strcat(folder_location_, "\", image_label);
    outputImg = imread(imagePath);
    hFig = figure;
    imshow(outputImg);
    
    % Set up callback for mouse click
    set(hFig, 'WindowButtonDownFcn', @(src, event) clickCallback(src, event, hFig));
    
    % Wait for user to click
    waitfor(hFig, 'UserData');
    
    % Retrieve the coordinates from 'UserData'
    coords = get(hFig, 'UserData');
    % converting to integer
    coords_int32 = int32(coords);

    % Append the new row to the cell array
    new_row = {image_label, coords_int32(1), coords_int32(2)};
    click_coordinates = [click_coordinates; new_row];
    
    % Close the figure
    close(hFig);
    progress_ = sprintf('progresss %.2f%%', (i/2000)*100);
    disp(progress_);
end

% Extract individual columns from the cell array
filename_col = click_coordinates(:, 1);
x_center_col = cell2mat(click_coordinates(:, 2));
y_center_col = cell2mat(click_coordinates(:, 3));

% Create a table with variable names
T = table(filename_col, x_center_col, y_center_col, 'VariableNames', {'filename', 'x_center', 'y_center'});

write_file_name_ = strcat("ML_label_data_set_", set_, ".csv"); 
writetable(T, write_file_name_);

%% Callback function to handle mouse clicks
function clickCallback(src, event, hFig)
    point = get(gca, 'CurrentPoint');
    x = point(1, 1);
    y = point(1, 2);
    
    % Draw a bright green marker at the clicked position
    hold on;
    plot(x, y, 'g+', 'MarkerSize', 10, 'LineWidth', 2);
    hold off;
    
    % Store the coordinates in the 'UserData' property of the figure
    set(hFig, 'UserData', [x, y]);
end
