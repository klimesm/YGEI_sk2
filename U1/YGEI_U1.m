clc; clear; close all; format longg;
% YGEI - Geoinformatika
% cviceni 1 - JPEG komprese
% Autor: Matej Klimes, Tomas Zbiral
% Datum: 1.10.2024

%% Insert variables for compression
% Load raster

ras1 = imread('snow_leopard.tif');

% Compression factor
q = 10;

% Type of transformation, Choose:
%                                'fft' for fast fourier transform
%                                'dct' for discrete cosine transform
Transform_Type = 'fft';

% Type of interpolation, Choose:
%                               'NearestNeighbour2x2', 
%                               'NearestNeighbour4x4',
%                               'Linear2x2', 
%                               'Linear4x4'
Int_down = 'NearestNeighbour2x2';
Int_up = 'NearestNeighbour2x2';

%% Check of input

% Check if image is in color or grayscale
[m,n,num] = size(ras1);
if num ==1
    % if grayscale, create all RGB channels equal
    ras1(:,:,2) = ras1(:,:,1);
    ras1(:,:,3) = ras1(:,:,1);
end

% Check if valid type of transformation was inserted
if strcmp(Transform_Type,'dct')
    ITransform_Type = 'idct';
elseif strcmp(Transform_Type,'fft')
    ITransform_Type = 'ifft';
else
disp('Type of transformation wasn''t inserted, ''dct'' will be used')
    Transform_Type = 'dct';
    ITransform_Type = 'idct';
end


%% 
% divide RGB channels
R = double(ras1(:,:,1));
G = double(ras1(:,:,2));
B = double(ras1(:,:,3));

% conversion of RGB to Y, C_B, C_R
Y = 0.2990 * R + 0.5870 * G + 0.1140 * B;
CB = -0.1687 * R - 0.3313 * G + 0.5000 *  B + 128;
CR = 0.5 * R - 0.4187 * G - 0.0813 * B + 128;

% downsampling
CB = resample_image(CB,'down',Int_down);
CR = resample_image(CR,'down',Int_down);

% Quantization matrices
Qy = [16 11 10 16 24 40 51 61
12 12 14 19 26 58 60 55
14 13 16 24 40 87 69 56
14 17 22 29 51 87 80 62
18 22 37 26 68 109 103 77
24 35 55 64 81 104 113 92
49 64 78 87 103 121 120 101
72 92 95 98 112 100 103 99];

Qc = [17 18 24 47 66 99 99 99
18 21 26 66 99 99 99 99
24 26 56 99 99 99 99 99
47 69 99 99 99 99 99 99
99 99 99 99 99 99 99 99
99 99 99 99 99 99 99 99
99 99 99 99 99 99 99 99
99 99 99 99 99 99 99 99];

% Adjust matrices according to q
Qy = 50*Qy/q;
Qc = 50*Qc/q;

% Compression
Yc = compression(Y,Qy,Transform_Type);
CBc = compression(CB,Qc,Transform_Type);
CRc = compression(CR,Qc,Transform_Type);

% Decompression
Yd = decompression(Yc,Qy,ITransform_Type);
CBd = decompression(CBc,Qc,ITransform_Type);
CRd = decompression(CRc,Qc,ITransform_Type);

% upsampling
CBd = resample_image(CBd,'up',Int_down);
CRd = resample_image(CRd,'up',Int_down);

% YCBCR to RGB
Rd = Yd + 1.4020*(CRd-128);
Gd = Yd - 0.3441*(CBd-128) - 0.7141 * (CRd-128);
Bd = Yd + 1.7720*(CBd-128) - 0.0001*(CRd-128);

% double na uint8
Ri = uint8(Rd);
Gi = uint8(Gd);
Bi = uint8(Bd);

% combine RGB to image
ras2(:,:,1) = Ri;
ras2(:,:,2) = Gi;
ras2(:,:,3) = Bi;

% standard deviations
dR = (R-Rd).^2;
dG = (G-Gd).^2;
dB = (B-Bd).^2;

sigR = sqrt(sum(sum(dR))/(m*n));
sigG = sqrt(sum(sum(dG))/(m*n));
sigB = sqrt(sum(sum(dB))/(m*n));

% show images for comparison
figure
subplot(1,2,1)
imshow(ras1)
subplot(1,2,2)
imshow(ras2)
