clc; clear; close all; format longg;
% YGEI - Geoinformatika
% uloha 2 - Vyhledani objektu v mapach
% Autor: Matej Klimes
% Datum: 15.10.2024
%% 
rgb = imread('images/map.jpg');
lab = rgb2lab(rgb);
l = lab(:,:,1);
im1 = uint8(l*2.55);
im2 = edge(im1);
imshow(im2)
%%
figure
subplot(3,2,1)
imshow(im1)
title('Grayscale')
subplot(3,2,2)
imshow(imsharpen(im1,'Radius',8,'Amount',10))
title('Sharpened grayscale')
subplot(3,2,3)
imshow(im2)
title('Edges')

%%

template_positions = load("positions.mat").positions;
template = imcrop(im1,template_positions(3,:));
subplot(3,2,4)
imshow(template)
corr = normxcorr2(template,im1);
subplot(3,2,5)
imshow(corr*255)
mincorr = 0.547;
[r,c] = size(corr);
count = 0;
figure(3)
imshow(rgb)
hold on
for row=1:r
    for col=1:c
        actual_coef = abs(corr(row,col));
        if actual_coef>=mincorr
            count = count+1;
            detections(count,1) = row;
            detections(count,2) = col;
            detections(count, 3) = actual_coef;
        end
    end
end

unique_detections = [];
min_distance = 10; 

for i = 1:size(detections, 1)
    current_detection = detections(i, 1:2);
    if isempty(unique_detections)
        unique_detections = current_detection;
    else
        distances = sqrt(sum((unique_detections - current_detection).^2, 2));
        if all(distances > min_distance)
            unique_detections = [unique_detections; current_detection];
        end
    end
end

for i = 1:size(unique_detections)
    rectangle('Position', [unique_detections(i, 2)-30 unique_detections(i, 1)-80 30 80], 'EdgeColor', 'r', 'LineWidth', 1)
end
    
    


                
