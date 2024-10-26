function Rt = mydct(R)
% mydct computes the 2D Discrete Cosine Transform (DCT) of an 8x8 matrix R.
% IN:
%    R  - An 8x8 matrix representing an image block (usually pixel values).
% OUT:
%    Rt - An 8x8 matrix of the DCT coefficients for the input block.

Rt = R;  % Initialize the output matrix Rt with the same size as R.

% Loop over rows of the output DCT matrix
for u = 0:7
    % Define Cu, the normalization factor for the u-th row.
    if u == 0
        Cu = sqrt(2)/2;  % Cu = sqrt(2)/2 for u = 0 (DC component)
    else
        Cu = 1;          % Cu = 1 for all other u (AC components)
    end
    
    % Loop over columns of the output DCT matrix
    for v = 0:7
        % Define Cv, the normalization factor for the v-th column.
        if v == 0
            Cv = sqrt(2)/2;  % Cv = sqrt(2)/2 for v = 0 (DC component)
        else
            Cv = 1;          % Cv = 1 for all other v (AC components)
        end
        
        % Initialize F to store the summation result (DCT coefficient).
        F = 0;
        
        % Loop over rows of the input matrix
        for x = 0:7
            % Loop over columns of the input matrix
            for y = 0:7
                % Compute the DCT contribution from each pixel (x, y).
                F = F + 1/4 * Cu * Cv *(R(x+1, y+1) * cos((2*x + 1) * u * pi / 16)*cos((2*y + 1) * v * pi / 16));
            end
        end
        % Assign the computed DCT coefficient F to the output matrix Rt.
        Rt(u+1, v+1) = F;
    end
end


end
