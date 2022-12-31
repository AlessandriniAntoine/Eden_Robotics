import os
import argparse
import time

import cv2

############################################ Arguments ###########################################

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--images", help="Path to folder that save the images.", type=str, nargs="?")
parser.add_argument("-n", "--num", help="Number of images to capture.", type=int, nargs="?")
parser.add_argument("-l", "--labels", help="Path to the text file containing the labels", type=str, nargs="?")
args = parser.parse_args()

folder = args.images or ''
num = args.num or 0
labels_path = args.labels or ''

############################################ Check Path Existance ###########################################

flag = True
if not os.path.exists(folder) :
    print('Invalide images path')
    flag = False
if not num : 
    print('Precise number of images to save.')
    flag = False

if not os.path.exists(labels_path) :
    print('Invalide labels path')
    flag = False
else :
    with open(labels_path,'r') as f:
        labels = f.read()
        labels = labels.split('\n')
        if not labels[-1]:
            labels = labels[:-1]
        labels.append('background')

############################################ Record ###########################################

if flag:
    cap = cv2.VideoCapture(0)
    for label in labels:
        imgnum = 0
        while imgnum < num : 
            ret, frame = cap.read()
            text_frame = frame.copy()
            cv2.putText(text_frame,f'Label : {label}, Image : {imgnum}/{num}', (100, 20), cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 0, 255))
            cv2.imshow('frame', text_frame)

            k  = cv2.waitKey(3)

            if k == ord('q'):
                break

            if k == ord('s'):
                imgname = os.path.join(folder,f'{time.strftime("%Y%m%d_%H%M%S")}.jpg')
                cv2.imwrite(imgname, frame)
                imgnum += 1


cap.release()
cv2.destroyAllWindows()

