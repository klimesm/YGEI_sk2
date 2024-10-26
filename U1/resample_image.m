function resampled_image = resample_image(Image,up_down,interp_type)
% function resampled_image = resample_image(Image,up_down,interp_type)
% function for image resampling using Nearest Neighbour or Linear interpolation
% IN:  Image           -- RGB or grayscale image
%      up_down         -- Upsampling ('up') or Downsampling ('down')
%      iterp_type      -- Type of interpolation: 'NearestNeighbour2x2', 'NearestNeighbour4x4','Linear2x2', 'Linear4x4'
%
% OUT: resampled_image -- resampled output image  


    if nargin<2
        up_down = 'down';
        interp_type = 'NearestNeighbour2x2';
    end
       
    if strcmp(interp_type,'NearestNeighbour2x2')
        scaleFactor = 0.5;
        interp_type = 'NearestNeighbour';
    elseif strcmp(interp_type,'NearestNeighbour4x4')
        scaleFactor = 0.25;
        interp_type = 'NearestNeighbour';
    elseif strcmp(interp_type,'Linear4x4')
        scaleFactor = 0.25;
        interp_type = 'Linear';
    elseif strcmp(interp_type,'Linear2x2')
        scaleFactor = 0.5;
        interp_type = 'Linear';
    else
        error('The specified interpolation type is not supported, supported types are: ''NearestNeighbour2x2'', ''NearestNeighbour4x4'',''Linear2x2'', ''Linear4x4''')

    end
    
    if strcmp(up_down,'up')
        scaleFactor = 1/scaleFactor;
    end

    % get image size and number of channels
    [rows, cols, numChannels] = size(Image);
    
    % new raster size
    newRows = round(rows * scaleFactor);
    newCols = round(cols * scaleFactor);


    % datatype of output image is the same as input image
    resampled_image = zeros(newRows, newCols, numChannels, 'like', Image);
    
   % for each channel
    for k = 1:numChannels
        % each row
        for i = 0:newRows-1
            % each column
            for j = 0:newCols-1
                
                % linear interpolation
                if strcmp(interp_type,'Linear')
                    if strcmp(up_down,'up')
                       orig_x = i / scaleFactor;
                        orig_y = j / scaleFactor;
    
                        % Find surrounding pixel indices
                        x1 = floor(orig_x);
                        x2 = min(x1 + 1, rows - 1);
                        y1 = floor(orig_y);
                        y2 = min(y1 + 1, cols - 1);
    
                        % Compute distance
                        dx = orig_x - x1;
                        dy = orig_y - y1;
    
                        % Ensure indices are within bounds
                        x1 = min(max(x1 + 1, 1), rows);
                        x2 = min(max(x2 + 1, 1), rows);
                        y1 = min(max(y1 + 1, 1), cols);
                        y2 = min(max(y2 + 1, 1), cols);
    
                        % Get the values of the four surrounding pixels
                        Q11 = double(Image(x1, y1, k));
                        Q12 = double(Image(x1, y2, k));
                        Q21 = double(Image(x2, y1, k));
                        Q22 = double(Image(x2, y2, k));
    
                        % Perform bilinear interpolation
                        R1 = (1 - dx) * Q11 + dx * Q21;
                        R2 = (1 - dx) * Q12 + dx * Q22;
                        resampled_image(i + 1, j + 1, k) = (1 - dy) * R1 + dy * R2;

                    elseif strcmp(up_down,'down')
                        resampled_image(i+1, j+1, k) = mean(mean(Image(i/scaleFactor+1:i/scaleFactor+1/scaleFactor, j/scaleFactor+1:j/scaleFactor+1/scaleFactor, k)));
                    else
                        error('Invalid parameter ''up_down''. Insert ''up'' or ''down''')
                    end

                    % nearest neighbour interpolation
                elseif strcmp(interp_type,'NearestNeighbour')
                    origRow = floor(i / scaleFactor)+1;
                    origCol = floor(j/ scaleFactor)+1;
                    resampled_image(i+1, j+1, k) = Image(origRow, origCol, k);
                end
            end
        end
    end


