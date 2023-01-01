import time
import random
import cv2
from detector import Detector

from parameters import *

cap = cv2.VideoCapture(0)

detector = Detector()
labels = ['pen','pencil','scissors','eraser']

prev_frame_time = 0
new_frame_time = 0

colors = [(random.randint(0, 255), random.randint(0,255), random.randint(0, 255)) for _ in labels]

while cap.isOpened():
    ret, frame = cap.read()
    frame = cv2.resize(frame,(img_size,img_size))

    # Make detections 
    detector.format_yolov5(frame)
    detector.detect()

    class_ids, confidences, boxes = detector.wrap_detection()


    for (classid, confidence, box) in zip(class_ids, confidences, boxes):
         cv2.rectangle(frame, box, colors[classid], 2)
         cv2.rectangle(frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), colors[classid], -1)
         cv2.putText(frame, labels[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255))

    new_frame_time = time.time()
    fps = round(1/(new_frame_time-prev_frame_time),2)
    prev_frame_time = new_frame_time
    cv2.putText(frame, f'FPS : {fps}Hz', (2, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)
    cv2.imshow('YOLO', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()