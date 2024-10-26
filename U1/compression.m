function Yc = compression(Y,Q,Transform_Type)
% COMPRESSION Compresses an input matrix using block-based transform
% and quantization.
% 
% Inputs:
%   Y - Input matrix to be compressed (e.g., image data)
%   Q - Quantization matrix used for compression
%   Transform_Type - Type of transform to apply ('dct' for Discrete Cosine Transform or 'fft' for Fast Fourier Transform)
%
% Output:
%   Yc - Compressed version of the input matrix Y

% Get the size of input
[m,n] = size(Y);

for i = 1:8:m-7
    for j = 1:8:n-7
        % Create submatrices
        YS = Y(i:i+7,j:j+7);
        if strcmp(Transform_Type,'dct')
            % DCT
            Yt = mydct(YS);
        elseif strcmp(Transform_Type,'fft')
            % FFT
            Yt = cooley_tukey_fft2d(YS);
        end
        % quantization
        Yq = Yt./Q;

        % round values
        Yqr = round(Yq);

        % Overwrite values of submatrix with compressed values
        Yc(i:i+7,j:j+7) = Yqr;
    end
end