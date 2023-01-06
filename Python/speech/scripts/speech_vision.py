import time
import argparse

import cv2
import threading

from recorder import SpeechRecognition
from tracker import Tracker
from detector import Detector

from parameters import * 


############################################ ARGPARSE ###########################################

parser = argparse.ArgumentParser()
parser.add_argument("-m", "--model", help="Name of the model", type=str, nargs="?")
args = parser.parse_args()

model_name = args.model or model_name
model_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'data','models',f'{model_name}.h5')

############################################ AUDIO ###########################################

def run_voice_command():
    global text,ok
    sr = SpeechRecognition(model_path=model_path)
    while ok: 
        sr.predict_mic()
        text = sr.text
    sr.terminate()

############################################ VIDEO ###########################################

def get_fps(prev_frame_time):
    new_frame_time = time.time()
    fps = round(1/(new_frame_time-prev_frame_time),2)
    prev_frame_time = new_frame_time

    return fps,prev_frame_time

def run_vision_command():
    global text, ok

    # init camera
    cap = cv2.VideoCapture(0)

    # init variables
    tracking,detecting = False, False
    prev_frame_time = 0

    # init tracker
    tracker = Tracker()

    # init detector 
    detector = Detector()

    while cap.isOpened():
        flag, frame = cap.read()
        if not flag :
            ok = False
            break

        frame = cv2.resize(frame,dimension)

        fps,prev_frame_time = get_fps(prev_frame_time)

        cv2.putText(frame, f'FPS : {fps}Hz, Command : {text}, Tracking : {tracking}, Detection : {detecting}', (2, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)

        if text in labels_obj and not detecting : 
            detecting = True
            label = text
        if text  == 'stop':
            tracking = False
            detecting = False
            tracker.tracker.clear()
        if text == 'close':
            ok = False
            break

        if detecting :
            detector.format_yolov5(frame)
            detector.detect()
            detector.get_box_label(label)
            if detector.flag:
                index = detector.result_confidences.index(max(detector.result_confidences))
                bbox = detector.result_boxes[index]
                tracker.init_tracker(frame,bbox)
                if tracker.flag:
                    tracking = True
                    detecting = False
                else :
                    print('Error while initializing tracker !')

        if tracking : 
            tracker.track(frame)
            if not tracker.flag :
                tracking = False

        cv2.imshow('Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): # quit
            ok = False
            break

    cap.release()
    cv2.destroyAllWindows()

############################################ THREADING ###########################################

ok,text = True,None

videoStreamThread = threading.Thread(target=run_vision_command)
videoStreamThread.start()

voiceStreamThread = threading.Thread(target=run_voice_command)
voiceStreamThread.start()
