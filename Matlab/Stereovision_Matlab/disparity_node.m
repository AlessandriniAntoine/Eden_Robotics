 %%%%%%%% Setup parameters %%%%%%%%%%%%%%
% key_press = waitforbuttonpress;
% value=double(get(gcf,'CurrentCharacter'))


row = 1;
column = 1;
M = 2.6547751e+03;

% load stereoParameters

[DISTANCE Size_x_pixel Size_y_pixel] = csvimport('Size_pixels.csv', 'columns', {'Distance', 'size_pixel_x', 'size_pixel_y'});

%%%%%%%%% Set up and open camera  %%%%%%%%%%%%%%%

Camera_name = '/dev/video0';
camera = webcam(Camera_name)

%%%%%%% Image processing
while true
    
    frame = snapshot(camera);
    frame = imresize(frame, [480, 1280]);

    frame_left = frame(:,1:640,:);
    frame_right = frame(:,641:1280,:);

%     disparityMap = disparity(frame_left,frame_right);
%     
%     inverse_disparity = 1./disparityMap;
%     inverse_disparity(isinf(inverse_disparity) | isnan(inverse_disparity)) = 0;
    
%     imshow(disparityMap,[0 127])
%     colormap jet
%     colorbar
    
    imshow(frame_left)

end

%%%%%%%% Close all %%%%%%%%%%%%%%
close
clear('camera');