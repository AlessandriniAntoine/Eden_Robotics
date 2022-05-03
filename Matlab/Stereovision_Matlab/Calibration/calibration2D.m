%% Parameters

number_images = 9;
depth = [ 25; 30; 35; 40; 45; 50; 55; 60; 65];
box_size = 1.55; %cm

Size_x_pixel = [];
Size_y_pixel = [];

% get stereo cameras parameters
load stereoParameters
%% Image Processing

for i = [1:1:number_images]
    
    frame_left = imread(strcat('images/2D/stereoLeft/imageL',int2str(depth(i)),'cm.png'));
    frame_right = imread(strcat('images/2D/stereoRight/imageR',int2str(depth(i)),'cm.png'));

    % Rectification
    [frame_left , frame_right] = rectifyStereoImages(frame_left, frame_right, stereoParams);
    
    [imagePoints,boardSize] = detectCheckerboardPoints(frame_left);
    frame_left = insertText(frame_left,imagePoints,1:size(imagePoints,1));
    frame_left = insertMarker(frame_left,imagePoints,'o','Color','red','Size',5);
    
%     axis ij;
%     axis manual;
%     [m n] = size(frame_left);  %%d1 is the image which I want to display
%     axis([0 m 0 n])
%     f1=figure
%     imshow(frame_left);
%     title(sprintf('Detected a %d x %d Checkerboard',boardSize));
%     coordinates_input = ginput(1);
%     row = round(coordinates_input(2));
%     column = round(coordinates_input(1));
%     fprintf('You clicked on pixel in row %d, column %d\n', row, column);
    
    x_pixel = (imagePoints(49,1)-imagePoints(1,1))/(8*box_size);
    y_pixel = (imagePoints(6,2)-imagePoints(1,2))/(5*box_size);
    Size_x_pixel = [Size_x_pixel ; x_pixel];
    Size_y_pixel = [Size_y_pixel ; y_pixel];
end 


%% CSV creation
Data = [depth Size_x_pixel Size_y_pixel];

cHeader = {'Distance' 'size_pixel_x' 'size_pixel_y'}; %dummy header
commaHeader = [cHeader;repmat({','},1,numel(cHeader))]; %insert commaas
commaHeader = commaHeader(:)';
textHeader = cell2mat(commaHeader); %cHeader in text with commas
%write header to file
fid = fopen('Size_pixels.csv','w'); 
fprintf(fid,'%s\n',textHeader)
fclose(fid)
%write data to end of file
dlmwrite('Size_pixels.csv',Data,'-append');
% 
% Data = [depth Size_x_pixel Size_y_pixel];
% clear; close all; clc;
% csvwrite('Size_pixel.csv',Data)
