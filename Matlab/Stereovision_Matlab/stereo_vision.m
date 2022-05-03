%% Correction

% get parameters
load stereoParameters

% Read image
frame_left = imread('imageL28.png');
frame_right = imread('imageR28.png');

% Display image before rectification
figure()
tiledlayout(2,1)
nexttile
imshow(frame_left)
nexttile
imshow(frame_right)

% Rectification
[rectification_left , rectification_right] = rectifyStereoImages(frame_left, frame_right, stereoParams);

% Display image after
figure()
tiledlayout(2,1)
nexttile
imshow(rectification_left)
nexttile
imshow(rectification_right)
%% Disparity

% Convert in gray image 
frameLeftGray  = rgb2gray(rectification_left);
frameRightGray = rgb2gray(rectification_right);
    
% Calculate disparity Map
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
inverse_disparity = 1./disparityMap;
inverse_disparity(isinf(inverse_disparity) | isnan(inverse_disparity)) = 0;

% Display disparity
figure;
imshow(disparityMap, [0, 63]);
title('Disparity Map');
colormap jet
colorbar

% Display disparity
figure;
imshow(inverse_disparity,[0,1.5]);
title('Inverse Disparity Map');
colormap jet
colorbar
%% Event

% Mouse clicking
frame = disparityMap;
axis ij;
axis manual;
[m n] = size(frame);  %%d1 is the image which I want to display
axis([0 m 0 n])
imshow(frame, [0, 63]);
title('Disparity Map');
colormap jet
colorbar
coordinates_input = ginput(1);
row = round(coordinates_input(2));
column = round(coordinates_input(1));
fprintf('You clicked on pixel in row %d, column %d\n', row, column);
