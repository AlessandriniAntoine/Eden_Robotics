import cv2
import numpy as np

from  parameters import *

class Detector:

    input_width : int = img_size
    input_height : int = img_size
    score_threshold :float = score_threshold
    nms_threshold : float = nms_threshold
    confidence_threshold : float = confidence_threshold

    weights : str = weigths_path

    def __init__(self):
        self.build_model()

    def build_model(self):
        self.net = cv2.dnn.readNet(self.weights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def format_yolov5(self,frame):
        row, col, _ = frame.shape
        _max = max(col, row)
        self.yolo_frame = np.zeros((_max, _max, 3), np.uint8)
        self.yolo_frame[0:row, 0:col] = frame

    def detect(self):
        blob = cv2.dnn.blobFromImage(self.yolo_frame, 1/255.0, size = (self.input_width, self.input_height), swapRB=True, crop=False)
        self.net.setInput(blob)
        self.detection = self.net.forward()

    def wrap_detection(self):
        class_ids = []
        confidences = []
        boxes = []

        rows = self.detection[0].shape[0]

        image_width, image_height, _ = self.yolo_frame.shape

        x_factor = image_width / self.input_width
        y_factor =  image_height / self.input_height

        for r in range(rows):
            row = self.detection[0][r]
            confidence = row[4]
            if confidence >= self.confidence_threshold:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes