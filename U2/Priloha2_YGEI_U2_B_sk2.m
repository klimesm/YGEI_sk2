clc; close all; clear variables; format longg; 

%% Úloha B

% Loading the image
image = imread('images/TM25_sk2.jpg');
image_R = uint8(image(:, :, 1));
image_G = uint8(image(:, :, 2));
image_B = uint8(image(:, :, 3));

% Convert image from RGB to lab
lab_image = rgb2lab(image);
image_l = uint8(lab_image(:, :, 1));
image_a = uint8(lab_image(:, :, 2));
image_b = uint8(lab_image(:, :, 3));

n_classes = 4;

% convert image from RGB to hsv
image_hsv = rgb2hsv(image);
image_h = uint8(image_hsv(:, :, 1));
image_s = uint8(image_hsv(:, :, 2));
image_v = uint8(image_hsv(:, :, 3));

image_types = {image_R, image_G, image_B, image_l, image_a, image_b, image_h, image_s, image_v};

% segment forest using k-means for all available channels
for i = 1:length(image_types)
    currentImage = image_types{i};
    single_image = im2single(currentImage);
    [L, C] = imsegkmeans(single_image, n_classes, NumAttempts=10);
    fname = sprintf("images/image_%d/segmented_image_all_classes.jpg", i);
    imwrite(mat2gray(L), fname); 
    for class = 1:n_classes
        classMask = (L == class);
        filename = sprintf('images/image_%d/class_%d.jpg', [i, class]);
        imwrite(classMask, filename);
    end
end




