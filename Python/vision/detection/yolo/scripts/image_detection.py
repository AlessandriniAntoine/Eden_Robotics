import time
import random
import cv2
from detector import Detector

from parameters import *

images = images

detector = Detector()
labels = ['pen','pencil','scissors','eraser']


colors = [(random.randint(0, 255), random.randint(0,255), random.randint(0, 255)) for _ in labels]

for image in images:
    t = time.time()
    frame = cv2.imread(image)
    frame = cv2.resize(frame,(img_size,img_size))

    # Make detections 
    detector.format_yolov5(frame)
    detector.detect()

    class_ids, confidences, boxes = detector.wrap_detection()


    for (classid, confidence, box) in zip(class_ids, confidences, boxes):
         cv2.rectangle(frame, box, colors[classid], 2)
         cv2.rectangle(frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), colors[classid], -1)
         cv2.putText(frame, labels[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255))

    fps = round(1/(time.time()-t),2)
    cv2.putText(frame, f'FPS : {fps}Hz', (2, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)
    cv2.imshow('YOLO', frame)
    k = cv2.waitKey(0)
    
    if k == ord('q'):
        break

cv2.destroyAllWindows()