function Yd = decompression(Y,Q,Transform_Type)
% DECOMPRESSION Decompresses an input matrix using block-based inverse transform
% and dequantization.
% 
% Inputs:
%   Y - Compressed matrix to be decompressed (e.g., quantized image data)
%   Q - Quantization matrix used for decompression
%   Transform_Type - Type of inverse transform to apply ('idct' for Inverse Discrete Cosine Transform or 'ifft' for Inverse Fast Fourier Transform)
%
% Output:
%   Yd - Decompressed version of the input matrix Y


% Get the size of input
[m,n] = size(Y);

for i = 1:8:m-7
    for j = 1:8:n-7

        % Create submatrices
        YS = Y(i:i+7,j:j+7);

        % dequantization
        YSd = YS.*Q;
        if strcmp(Transform_Type,'idct')
        % IDCT
        Yt = myidct(YSd);
        elseif strcmp(Transform_Type,'ifft')
        % IFFT
        Yt = cooley_tukey_ifft2d(YSd);
        end

        % Overwrite values of submatrix with decompressed values
        Yd(i:i+7,j:j+7) = real(Yt);

    end
end