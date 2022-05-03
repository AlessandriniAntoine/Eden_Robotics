%% Parameters

images_number = 7;
row = 0;
column = 0;
DEPTH = [30 40 50 60 70 80 90];

% get stereo cameras parameters
load stereoParameters
%% Select Pixel 

frame_left = imread(strcat('images/depth/stereoLeft/imageL',int2str(DEPTH(8)),'cm.png'));
frame_right = imread(strcat('images/depth/stereoRight/imageR',int2str(DEPTH(8)),'cm.png'));

% Rectification
[rectification_left , rectification_right] = rectifyStereoImages(frame_left, frame_right, stereoParams);


% Convert in gray image 
frameLeftGray  = rgb2gray(rectification_left);
frameRightGray = rgb2gray(rectification_right);

% Calculate disparity Map
disparityMap = disparitySGM(frameLeftGray, frameRightGray);

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

%% Creation matrix 

INVERSE_disparity = [];
DISPARITY =[];

% Create matrix
for i = [1:1:images_number]
    frame_left = imread(strcat('images/depth/stereoLeft/imageL',int2str(DEPTH(i)),'cm.png'));
    frame_right = imread(strcat('images/depth/stereoRight/imageR',int2str(DEPTH(i)),'cm.png'));

    % Rectification
    [rectification_left , rectification_right] = rectifyStereoImages(frame_left, frame_right, stereoParams);


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
    
    INVERSE_disparity = [INVERSE_disparity inverse_disparity(row,column)];
    DISPARITY = [DISPARITY disparityMap(row,column)];
    i
end 


%% Display

t = tiledlayout(1,2);
nexttile
plot(DISPARITY,DEPTH)
xlabel('Disparity')
ylabel('Depth')
title(sprintf('Relation between depth \nand corresponding disparity'))
nexttile
plot(INVERSE_disparity,DEPTH)
xlabel('Inverse disparity (1/disparity)')
ylabel('Depth')
title(sprintf('Relation between depth \nand corresponding inverse disparity'))

%% Lineare Regression

p = polyfit(INVERSE_disparity,DEPTH,1)
x1 = linspace(0,0.05);
y1 = polyval(p,x1);
figure
plot(INVERSE_disparity,DEPTH,'*')
hold on
plot(x1,y1)
hold off

depth_coefficiant = p(1)