clc; close all; clear variables; format longg;

% Load the mask
mask = imread('images/image_6/class_2.jpg'); 
mask = imbinarize(mask); 

% Window size and threshold for deleting lines (roads)
window_size = 80;
threshold = 0.17; 

[height, width] = size(mask);
filtered_mask = mask; 
skip_next = false; % skip the next window if the current one passes the 10% threshold

for i = 1:window_size:height
    for j = 1:window_size:width
        x_end = min(i + window_size - 1, height);
        y_end = min(j + window_size - 1, width);
        
        window = mask(i:x_end, j:y_end);
        % count white pixels
        white_pixel_count = sum(window(:));
        
        if white_pixel_count / numel(window) >= threshold
            skip_next = true;
        else
            % if below threshold, set all pixels black
            filtered_mask(i:x_end, j:y_end) = 0;
            skip_next = false; 
        end
       
        if skip_next
            j = j + window_size; 
        end
    end
end

figure(1)
imshow(filtered_mask);

output_mask = uint8(filtered_mask) * 255;
imwrite(output_mask, 'images/filtered_mask.jpg');


% morphological closing to compact forest areas
se = strel('disk', 10); 
closed_mask = imclose(filtered_mask, se);

% fill remaining holes in the forest area
compacted_mask = imfill(closed_mask, 'holes');
imshow(compacted_mask);
imwrite(uint8(compacted_mask) * 255, 'compacted_forest_mask.jpg');

mask1 = imread('mask_GraphCut.jpg'); % mask from graph cut
mask2 = imread('compacted_forest_mask.jpg'); 
mask1 = imbinarize(mask1);
mask2 = imbinarize(mask2);
combined_mask = mask1 | mask2;
imshow(combined_mask);
title('Combined Mask (Logical OR)');
imwrite(uint8(combined_mask) * 255, 'combined_mask.jpg');


compacted_mask = imread('combined_mask.jpg'); 
compacted_mask = imbinarize(compacted_mask); 

map = imread('images/TM25_sk2.jpg');

if size(map, 3) == 3
    compacted_mask = repmat(compacted_mask, [1 1 3]);
end

% apply the mask to the map
masked_map = map;
masked_map(~compacted_mask) = 0; 

imshow(masked_map);

imwrite(masked_map, 'masked_map.jpg');