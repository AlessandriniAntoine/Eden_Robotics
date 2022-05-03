function myLoopingFcn() 
%%%%%%%%% Setup parameters %%%%%%%%%%%%%%

global KEY_IS_PRESSED
KEY_IS_PRESSED = 0;
row = 1;
column = 1;
M = 2.6547751e+03;

load stereoParameters

%%%%%%%%% Set up and open camera  %%%%%%%%%%%%%%%

Camera_name = '/dev/video2';
camera = webcam(Camera_name)

%%%%%%%% Call function to close if key press %%%%%%%%%%%

gcf
set(gcf, 'KeyPressFcn', @myKeyPressFcn)

%%%%%%% Image processing
while ~KEY_IS_PRESSED
    
    frame = snapshot(camera);
    frame = imresize(frame, [480, 1280]);

    frame_left = frame(:,1:640,:);
    frame_right = frame(:,641:1280,:);
    

    % Rectification
    [rectification_left , rectification_right] = rectifyStereoImages(frame_left, frame_right, stereoParams);


    % Convert in gray image 
    frameLeftGray  = rgb2gray(rectification_left);
    frameRightGray = rgb2gray(rectification_right);

    % Calculate disparity Map
    disparityMap = disparitySGM(frameLeftGray, frameRightGray);
    inverse_disparity = 1./disparityMap;
    inverse_disparity(isinf(inverse_disparity) | isnan(inverse_disparity)) = 0;
    
    imshow(disparityMap, [0 127])
    colormap jet 
    colorbar
    
%     % select the object
%     frame = disparityMap;
%     axis ij;
%     axis manual;
%     [m n] = size(frame); %     imshow(disparityMap,[0 127])
% %     colormap jet
% %     colorbar
%  %%d1 is the image which I want to display
%     axis([0 m 0 n])
%     imshow(frame, [0, 127]);
%     title('Disparity Map');
%     colormap jet
%     colorbar
%     coordinates_input = ginput(1);
%     row = round(coordinates_input(2));
%     column = round(coordinates_input(1));
%     fprintf('You clicked on pixel in row %d, column %d\n', row, column);
% 
%     depth = round(M*inverse_disparity(row,column)-1,2)
end

%%%%%%%% Close all %%%%%%%%%%%%%%
close
clear('camera');

function myKeyPressFcn(hObject, event)
global KEY_IS_PRESSED
KEY_IS_PRESSED  = 1;
disp('key is pressed')