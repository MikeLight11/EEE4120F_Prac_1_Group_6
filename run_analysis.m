% =========================================================================
% Practical 1: 2D Convolution Analysis
% =========================================================================
%
% GROUP NUMBER: 6
%
% MEMBERS:
%   - Member 1 Michael Lighton, LGHMIC003
%   - Member 2 Glen Jones, JNSGLE007

%% ========================================================================
%  PART 3: Testing and Analysis
%  ========================================================================
%
% Compare the performance of manual 2D convolution (my_conv2) with MATLAB's
% built-in conv2 function (inbuilt_conv2).

function run_analysis()
    % TODO1:
    % Load all the sample images from the 'sample_images' folder
    images{1} = imread('sample_images/image_128x128.png');
    images{2} = imread('sample_images/image_256x256.png');
    images{3} = imread('sample_images/image_512x512.png');
    images{4} = imread('sample_images/image_1024x1024.png');
    images{5} = imread('sample_images/image_2048x2048.png');

    image_names = {'image_128x128', 'image_256x256', 'image_512x512', 'image_1024x1024', 'image_2048x2048'};
    
    % TODO2:
    % Define edge detection kernels (Sobel kernel)
    Gx = [-1 0 1; -2 0 2; -1 0 1]; % Sobel kernel for x-direction
    Gy = [1 2 1; 0 0 0; -1 -2 -1]; % Sobel kernel for y-direction
    
    results = struct();
    
    % TODO3:
    % For each image, perform the following:
    for k = 1:length(images)
        %   a. Measure execution time of my_conv2
        tic;
        manual_output = my_conv2(images{k},Gx,Gy,'full');
        time_manual = toc;
        %   b. Measure execution time of inbuilt_conv2
        tic;
        inbuilt_output = inbuilt_conv2(images{k},Gx,Gy,'full');
        time_builtin = toc;
        %   c. Compute speedup ratio
        speed_up_ratio = time_manual / time_builtin;
        %   d. Verify output correctness (compare results)
        
        %   e. Store results (image name, time_manual, time_builtin, speedup)
        results(k).image_name = image_names{k};
        results(k).time_manual  = time_manual;
        results(k).time_builtin = time_builtin;
        results(k).speedup      = speed_up_ratio;
        %   f. Plot and compare results
        %   g. Visualise the edge detection results(Optional)
        figure;
        subplot(1,3,1), imshow(images{k}), title('Original');
        subplot(1,3,2), imshow(manual_output, []), title('Manual Conv');
        subplot(1,3,3), imshow(inbuilt_output, []), title('Built-in Conv');
    end

    % Print Header
    fprintf('\n%-20s | %-12s | %-12s | %-10s | %-10s\n', ...
        'Image Name', 'Manual (s)', 'Built-in (s)', 'Speedup');
    fprintf('%s\n', repmat('-', 1, 75));
    
    % Loop through results struct to print each row
    for k = 1:length(results)
        % Corrected printf for rows
        fprintf('%-20s | %-12.4f | %-12.4f | %-10.2f | %-10.2e\n', ...
            results(k).image_name, ...
            results(k).time_manual, ...
            results(k).time_builtin, ...
            results(k).speedup);
    end
        
    
end


%% ========================================================================
%  PART 1: Manual 2D Convolution Implementation
%  ========================================================================
%
% REQUIREMENT: You may NOT use built-in convolution functions (conv2, imfilter, etc.)

% TODO: Implement manual 2D convolution using Sobel Operator(Gx and Gy)
% output - Convolved image result (grayscale)
function output = my_conv2(image, Gx, Gy, output_mode)
    grey_image = double(rgb2gray(image));
    
    % Standard convolution flips the kernel 180 degrees
    Gx = rot90(Gx, 2);
    Gy = rot90(Gy, 2);
    
    % Get dimensions
    [rows, cols] = size(grey_image);
    [kernel_Row, kernel_Col] = size(Gx);
    
    % Padding based on mode
    switch lower(output_mode)
        case 'full'
            padRow = kernel_Row - 1; 
            padCol = kernel_Col - 1;
        case 'same'
            padRow = floor(kernel_Row/2);
            padCol = floor(kernel_Col/2);
        case 'valid'
            padRow = 0;
            padCol = 0;
    end
    
    % Pad the image
    paddedImage = padarray(grey_image, [padRow padCol], 0);
    
    % Calculate output dimensions based on padding
    out_rows = size(paddedImage, 1) - kernel_Row + 1;
    out_cols = size(paddedImage, 2) - kernel_Col + 1;
    
    % Initialize gradients
    gradient_x = zeros(out_rows, out_cols);
    gradient_y = zeros(out_rows, out_cols);
    
    % Perform convolution
    % Loop through the calculated output dimensions
    for i = 1:out_rows
        for j = 1:out_cols
            % Extract region matching the kernel size
            region = paddedImage(i:i+kernel_Row-1, j:j+kernel_Col-1);
            gradient_x(i, j) = sum(sum(region .* Gx));
            gradient_y(i, j) = sum(sum(region .* Gy));
        end
    end
    
    output = sqrt(gradient_x.^2 + gradient_y.^2);
end

%% ========================================================================
%  PART 2: Built-in 2D Convolution Implementation
%  ========================================================================
%   
% REQUIREMENT: You MUST use the built-in conv2 function

% TODO: Use conv2 to perform 2D convolution
% output - Convolved image result (grayscale)
function output = inbuilt_conv2(image, Gx, Gy, output_mode) %Add necessary input arguments
    % Convert image to double for correct math
    image = double(rgb2gray(image));

    % Convolve with Sobel kernels
    conv_x = conv2(image, Gx, output_mode);
    conv_y = conv2(image, Gy, output_mode);

    % Gradient magnitude
    output = sqrt(conv_x.^2 + conv_y.^2);
end

