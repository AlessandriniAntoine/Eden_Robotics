 %%%%%%%% Setup parameters %%%%%%%%%%%%%%
% key_press = waitforbuttonpress;
% value=double(get(gcf,'CurrentCharacter'))


row = 1;
column = 1;
M = 2.6547751e+03;

load stereoParameters

[DISTANCE Size_x_pixel Size_y_pixel] = csvimport('Size_pixels.csv', 'columns', {'Distance', 'size_pixel_x', 'size_pixel_y'});

%%%%%%%%% Set up and open camera  %%%%%%%%%%%%%%%

Camera_name = '/dev/video2';
camera = webcam(Camera_name)

%%%%%%% Image processing
while true
    
     key_press = waitforbuttonpress;
     value=double(get(gcf,'CurrentCharacter'));

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
    
    imshow(disparityMap,[0 127])
    colormap jet
    colorbar

    
    if value == 115
        imshow(frame_left)
        % select the object
        frame = frame_left;
        axis ij;
        axis manual;
        [m n] = size(frame_left);  %%d1 is the image which I want to display
        axis([0 m 0 n])
        f1=figure
        imshow(disparityMap,[0 127])
        colormap jet
        colorbar
        coordinates_input = ginput(1);
        row = round(coordinates_input(2));
        column = round(coordinates_input(1));
        fprintf('You clicked on pixel in row %d, column %d\n', row, column);
        
        [y, x] = size(frameLeftGray);
        x = column - x/2
        y =  y/2 - row  ;
        
        depth = round(M*inverse_disparity(row,column)-1,2);
        dist    = abs(DISTANCE - depth);
        minDist = min(dist);
        idx = find(dist == minDist);
        x_coefficiant = Size_x_pixel(idx)
        y_coefficiant = Size_y_pixel(idx);
        x = x/x_coefficiant-1;
        y = y/y_coefficiant ;
        fprintf(1, '  x  :  %G  cm\n ',x);
        fprintf(1, ' y  :  %G  cm\n ',y);
        fprintf(1, ' z  :  %G  cm\n\n ',depth);
        close(f1)

    elseif value == 113
        break
    end
    
    value = 0;

end

%%%%%%%% Close all %%%%%%%%%%%%%%
close
clear('camera');