# import cv2
# import time

# ################ Define camera parameters #############################

# frame_size = [640,480]

# cap = cv2.VideoCapture(2)
# cap.set(3, frame_size[0]*2)
# cap.set(4, frame_size[1])

# depth = [20, 25, 30, 35,40,45,50,55,60,65]

# num = 0

# while cap.isOpened(): 

# ################# Stero cameras ############################
#     succes1, image = cap.read()

#     frame_left = image[:,:int(frame_size[0])]
#     frame_right = image[:,int(frame_size[0]):frame_size[0]*2]

#     if succes1 :
#         k = cv2.waitKey(5)

#         if k == ord('q'):
#             break

#         elif k == ord('w'): # wait for 's' key to save and exit

#             cv2.imwrite('Calibration/images/2D/stereoLeft/imageL' + str(depth[num]) + 'cm.png', frame_left)
#             cv2.imwrite('Calibration/images/2D/stereoRight/imageR' + str(depth[num]) + 'cm.png', frame_right)
#             print("images" +str(num), " saved!")
#             num += 1

#         cv2.imshow('Left',frame_left)
#         cv2.imshow('Right',frame_right)
        

# cap.release()

# cv2.destroyAllWindows()

import numpy as np 
import cv2

frame_size = [1280,480]

# Camera paramters to undistort and rectify images
cv_file =  cv2.FileStorage()
cv_file.open('stereoMap.xml',cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()


#open both cameras
cap = cv2.VideoCapture(1)
cap.set(3, frame_size[0])
cap.set(4, frame_size[1])

while cap.isOpened():

    succes, image = cap.read()

    frame_left = image[:,:int(frame_size[0]/2)]
    frame_right = image[:,int(frame_size[0]/2):frame_size[0]]

    # Undistort and rectify images
    frame_right1 = cv2.remap(frame_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    frame_left1 = cv2.remap(frame_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    # Show the frames
    # cv2.imshow("frame left",frame_left)
    cv2.imshow("frame right", frame_right)
    cv2.imshow("frame left rectified", frame_left)

    
    #Hit q to close the window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()
print(frame_left.shape)
