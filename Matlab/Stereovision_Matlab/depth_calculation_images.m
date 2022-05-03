%% Parameters

row = 0;
column = 0;
M = 2.6547751e+03;

% get stereo cameras parameters
load stereoParameters
%% Select Pixel 

frame_left = imread(strcat('images/depth/tests/stereoLeft/imageL',int2str(56),'cm.png'));
frame_right = imread(strcat('images/depth/tests/stereoRight/imageR',int2str(56),'cm.png'));

% Rectification
[frame_left , frame_right] = rectifyStereoImages(frame_left, frame_right, stereoParams);


% Convert in gray image 
frameLeftGray  = rgb2gray(frame_left);
frameRightGray = rgb2gray(frame_right);

% Calculate disparity Map
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
inverse_disparity = 1./disparityMap;
inverse_disparity(isinf(inverse_disparity) | isnan(inverse_disparity)) = 0;

% select the object
frame = disparityMap;
axis ij;
axis manual;
[m n] = size(frame);  %%d1 is the image which I want to display
axis([0 m 0 n])
imshow(frame, [0, 127]);
title('Disparity Map');
colormap jet
colorbar
coordinates_input = ginput(1);
row = round(coordinates_input(2));
column = round(coordinates_input(1));
fprintf('You clicked on pixel in row %d, column %d\n', row, column);

depth = M*inverse_disparity(row,column)-1
