function [disparityMap] = disparity(frame_left,frame_right)
    load stereoParameters
     % Rectification
    [rectification_left , rectification_right] = rectifyStereoImages(frame_left, frame_right, stereoParams);

    % Convert in gray image 
    frameLeftGray  = rgb2gray(rectification_left);
    frameRightGray = rgb2gray(rectification_right);
    
    % Calculate disparity Map
    disparityMap = disparitySGM(frameLeftGray, frameRightGray);
end 