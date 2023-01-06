from random import randint
import cv2
from parameters import * 

class Tracker:

    def __init__(self):

        self.type = tracker_type
        self.types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
        self.create_tracker_by_name()
        
        self.flag = True
        self.colors = (randint(0, 255), randint(0,255), randint(0, 255))

    def create_tracker_by_name(self):
        if self.type == self.types[0]:
            self.tracker = cv2.legacy.TrackerBoosting_create()
        elif self.type == self.types[1]:
            self.tracker = cv2.legacy.TrackerMIL_create()
        elif self.type == self.types[2]:
            self.tracker = cv2.legacy.TrackerKCF_create()
        elif self.type == self.types[3]:
            self.tracker = cv2.legacy.TrackerTLD_create()
        elif self.type == self.types[4]:
            self.tracker = cv2.legacy.TrackerMedianFlow_create()
        elif self.type == self.types[5]:
            self.tracker = cv2.legacy.TrackerMOSSE_create()
        elif self.type == self.types[6]:
            self.tracker = cv2.legacy.TrackerCSRT_create()
        else:
            self.tracker = None
            print('Invalid name! Available trackers: ')
            for t in self.types:
                print(t)

    def init_tracker(self,frame,bbox):
        self.create_tracker_by_name()
        self.flag = self.tracker.init(frame, bbox)

    def track(self,frame):
        self.flag, bbox = self.tracker.update(frame)
        if self.flag == True:
            (x, y, w, h) = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), self.colors, 2)