import os
import sys
import time
from random import randint
import numpy as np

import cv2
import matplotlib.pyplot as plt

tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
def create_tracker_by_name(tracker_type):
    if tracker_type == tracker_types[0]:
        tracker = cv2.legacy.TrackerBoosting_create()
    elif tracker_type == tracker_types[1]:
        tracker = cv2.legacy.TrackerMIL_create()
    elif tracker_type == tracker_types[2]:
        tracker = cv2.legacy.TrackerKCF_create()
    elif tracker_type == tracker_types[3]:
        tracker = cv2.legacy.TrackerTLD_create()
    elif tracker_type == tracker_types[4]:
        tracker = cv2.legacy.TrackerMedianFlow_create()
    elif tracker_type == tracker_types[5]:
        tracker = cv2.legacy.TrackerMOSSE_create()
    elif tracker_type == tracker_types[6]:
        tracker = cv2.legacy.TrackerCSRT_create()
    else:
        tracker = None
        print('Invalid name! Available trackers: ')
        for t in tracker_types:
            print(t)
    return tracker

def select_bbox(cap,tracker):
    while True:
        flag, frame = cap.read()
        if not flag:
            break

        cv2.imshow('Tracking', frame)
        if cv2.waitKey(1) & 0XFF == ord('s'): 
            bbox = cv2.selectROI(frame) # region of interest
            flag = tracker.init(frame, bbox)
            if not flag:
                print('Error while initializing tracker.')
                return None
            return bbox
        
        if cv2.waitKey(1) & 0XFF == ord('q'): 
            break
    return None

def track(cap,tracker,bbox):
    prev_frame_time, new_frame_time = 0, 0
    colors = (randint(0, 255), randint(0,255), randint(0, 255))
    while True:
        flag, frame = cap.read()
        if not flag:
            break

        flag, bbox = tracker.update(frame)
        if flag == True:
            (x, y, w, h) = [int(v) for v in bbox]
            print(f'Width : {w}, Height : {h}')
            cv2.rectangle(frame, (x, y), (x + w, y + h), colors, 2)
        else:
            cv2.putText(frame, 'Tracking failure!', (100,80), cv2.FONT_HERSHEY_SIMPLEX, .75, (0,0,255))
            bbox = cv2.selectROI(frame) # region of interest
            tracker = create_tracker_by_name(tracker_type)
            flag = tracker.init(frame, bbox)
            if not flag:
                print('Error while initializing tracker.')
                break
            
        new_frame_time = time.time()
        fps = round(1/(new_frame_time-prev_frame_time),2)
        prev_frame_time = new_frame_time
        cv2.putText(frame,f'Tracker : {tracker_type}, FPS : {fps}HZ', (100, 20), cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 0, 255))

        cv2.imshow('Tracking', frame)
        if cv2.waitKey(1) & 0XFF == ord('q'): 
            break
        
    cap.release()
    cv2.destroyAllWindows()




# init tracker
tracker_type = 'KCF'
tracker = create_tracker_by_name(tracker_type)

# init camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('Error while opening the camera!')
    sys.exit()
flag, frame = cap.read()
if not flag:
    print('Erro while loading the frame!')
    sys.exit()

bbox = select_bbox(cap,tracker)
track(cap,tracker,bbox)
