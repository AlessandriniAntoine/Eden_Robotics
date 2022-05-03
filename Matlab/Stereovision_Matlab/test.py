import cv2 as cv
import numpy as np

frame_size = (480,640)

camera_left_matrix = np.mat('[425.1630, 0, 346.8159];[0, 425.0656, 263.8883];[0.00000000e+00,0.00000000e+00,1.00000000e+00]')
camera_right_matrix = np.mat('[[424.5805, 0, 352.7144];[0, 424.6613, 253.4969];[0.00000000e+00,0.00000000e+00,1.00000000e+00]]')

camera_left_distortion = np.mat('[[ -0.4059, 0.2057, -0.0004, 0.0007, -0.0638]]')
camera_right_distortion = np.mat('[[ -0.4002, 0.1894, -0.0005, -0.0003, -0.0504]]')

rotation = np.mat('[[0.9999, -0.0004, 0.0156];[0.0003, 1.0000, 0.0081];[ -0.0156, -0.0081, 0.9998]]')
translation = np.mat('[[-57.9018];[-0.1118];[ 0.4443]]')

rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv.stereoRectify(camera_left_matrix, camera_left_distortion, camera_right_matrix, camera_right_distortion, frame_size, rotation, translation, None, None, None, None, None, cv.CALIB_ZERO_DISPARITY, 0)

stereoMapL = cv.initUndistortRectifyMap(camera_left_matrix, camera_left_distortion, rectL, projMatrixL, frame_size, cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(camera_right_matrix, camera_right_distortion, rectR, projMatrixR, frame_size, cv.CV_16SC2)

print("Saving parameters!")
cv_file = cv.FileStorage('stereoMap.xml', cv.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x',stereoMapL[0])
cv_file.write('stereoMapL_y',stereoMapL[1])
cv_file.write('stereoMapR_x',stereoMapR[0])
cv_file.write('stereoMapR_y',stereoMapR[1])

cv_file.release()