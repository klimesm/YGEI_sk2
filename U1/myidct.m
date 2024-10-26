function Rt = myidct(R)
% myidct computes the 2D Inverse Discrete Cosine Transform (IDCT) of an 8x8 matrix R.
% IN:
%    R  - An 8x8 matrix representing DCT coefficients (frequency domain).
% OUT:
%    Rt - An 8x8 matrix of reconstructed pixel values (spatial domain).

Rt = R;  % Initialize the output matrix Rt with the same size as R.

% Loop over rows of the output spatial matrix
for x = 0:7
    % Loop over columns of the output spatial matrix
    for y = 0:7
        % Initialize F to store the summation result (reconstructed pixel value).
        F = 0;
        
        % Loop over rows of the input DCT coefficient matrix
        for u = 0:7
            % Define Cu, the normalization factor for the u-th row.
            if u == 0
                Cu = sqrt(2)/2;  % Cu = sqrt(2)/2 for u = 0 (DC component)
            else
                Cu = 1;          % Cu = 1 for all other u (AC components)
            end
            
            % Loop over columns of the input DCT coefficient matrix
            for v = 0:7
                % Define Cv, the normalization factor for the v-th column.
                if v == 0
                    Cv = sqrt(2)/2;  % Cv = sqrt(2)/2 for v = 0 (DC component)
                else
                    Cv = 1;          % Cv = 1 for all other v (AC components)
                end
                
                % Compute the inverse DCT contribution from each coefficient (u, v).
                F = F + 1/4 * Cu * Cv * (R(u+1, v+1) * cos((2*x + 1) * u * pi / 16) * cos((2*y + 1) * v * pi / 16));
            end
        end
        
        % Assign the reconstructed pixel value F to the output matrix Rt.
        Rt(x+1, y+1) = F;
    end
end

end
