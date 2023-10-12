% Directory containing the images
image_dir = 'D:\OneDrive - UTS\Documents\Spring 2023\Sensors and Control\Assessments\Assessment 2 - Mid-Reveiw Report\Code- Camera Calibration Real Distance\Data2_Soft_pullback_1\Images';
images = dir(fullfile(image_dir, '*.jpg')); % Assuming images are in jpg format

% Define expected centroid locations for the two target regions
expected_centroid1 = [379, 63];
expected_centroid2 = [380, 129];

% Define RGB thresholds
R_min = 0; R_max = 65;
G_min = 180; G_max = 260;
B_min = 175; B_max = 260;

total_pixel_distance = 0;
valid_image_count = 0;

% Iterate over all images in the directory
for k = 1:length(images)
    image_path = fullfile(image_dir, images(k).name);
    
    fprintf('Processing image: %s\n', images(k).name); % Print the image name
    
    % Load the image
    img = imread(image_path);

    % RGB thresholding
    R_mask = (img(:,:,1) >= R_min) & (img(:,:,1) <= R_max);
    G_mask = (img(:,:,2) >= G_min) & (img(:,:,2) <= G_max);
    B_mask = (img(:,:,3) >= B_min) & (img(:,:,3) <= B_max);
    blue_mask = R_mask & G_mask & B_mask;

    % Label the regions
    labeled_image = bwlabel(blue_mask);
    stats = regionprops(labeled_image, 'Centroid', 'PixelList');

    if isempty(stats)
        fprintf('No regions detected in image %s. Skipping...\n', images(k).name);
        continue;
    end

    % Find the two regions closest to the given centroids
    distances1 = cellfun(@(c) norm(c - expected_centroid1), {stats.Centroid});
    distances2 = cellfun(@(c) norm(c - expected_centroid2), {stats.Centroid});

    [~, idx1] = min(distances1);
    [~, idx2] = min(distances2);

    if isempty(idx1) || isempty(idx2)
        fprintf('Could not find both regions in image %s. Skipping...\n', images(k).name);
        continue;
    end

    % Compute the average of the centroids' borders
    avg_border1 = mean(stats(idx1).PixelList, 1);
    avg_border2 = mean(stats(idx2).PixelList, 1);

    % Calculate the distance in pixels between these average points
    pixel_distance = norm(avg_border1 - avg_border2);

    total_pixel_distance = total_pixel_distance + pixel_distance;
    valid_image_count = valid_image_count + 1;
end

if valid_image_count == 0
    fprintf('No valid regions detected in any of the images.\n');
    return;
end

average_pixel_distance = total_pixel_distance / valid_image_count;
real_distance_mm = 5;
distance_per_pixel_mm = real_distance_mm / average_pixel_distance;

fprintf('Average distance in pixels between the regions: %f\n', average_pixel_distance);
fprintf('Real-world distance per pixel: %f mm/pixel\n', distance_per_pixel_mm);
