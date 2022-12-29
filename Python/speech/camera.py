import time
import threading
from random import randint
import numpy as np
import os
import argparse

import cv2
import pyaudio
import tensorflow as tf
from tensorflow.keras import models


############################################ Define ArgParse ###########################################

parser = argparse.ArgumentParser()
parser.add_argument("-m", "--model", help="Number of the model", type=int, nargs="?")
args = parser.parse_args()

############################################ Define Variables ###########################################

model_number = args.model or 8

tracker_type = 'CSRT'
dimension = (640,640)

FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

############################################ Define Parameters ###########################################

# paths
DIR_PATH = os.path.dirname(__file__)
paths = {
    'FILES_PATH' : os.path.join(DIR_PATH,'data','audio','test'),
    'MODELS_PATH' : os.path.join(DIR_PATH,'data','models'),
}

# init variables
ok = True
success,text = False,None

# fix variables
model = models.load_model(os.path.join(paths['MODELS_PATH'],f'model_{model_number}.h5'))
label_names = os.listdir(paths['FILES_PATH'])
label_names.sort()
p = pyaudio.PyAudio()
tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']

############################################ SPEECH RECOGNITION ###########################################

def record_audio():
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=FRAMES_PER_BUFFER
    )

    frames = []
    seconds = 1
    for _ in range(int(RATE / FRAMES_PER_BUFFER * seconds)):
        data = stream.read(FRAMES_PER_BUFFER)
        frames.append(data)

    stream.stop_stream()
    stream.close()

    return np.frombuffer(b''.join(frames), dtype=np.int16)

def terminate():
    p.terminate()

def get_spectrogram(waveform):
  spectrogram = tf.signal.stft(
      waveform, frame_length=255, frame_step=128)
  spectrogram = tf.abs(spectrogram)
  spectrogram = spectrogram[..., tf.newaxis]
  return spectrogram

def preprocess_audiobuffer(waveform):
    """
    waveform: ndarray of size (16000, )
    
    output: Spectogram Tensor of size: (1, `height`, `width`, `channels`)
    """
    #  normalize from [-32768, 32767] to [-1, 1]
    waveform =  waveform / 32768
    waveform = tf.convert_to_tensor(waveform, dtype=tf.float32)
    spectogram = get_spectrogram(waveform)
    # add one dimension
    return tf.expand_dims(spectogram, 0)

def predict_mic():
    audio = record_audio()
    spec = preprocess_audiobuffer(audio)
    prediction = model(spec)
    label_pred = np.argmax(prediction, axis=1)
    return label_names[label_pred[0]]

def run_voice_command():
    global success,text,ok
    while ok: 
        text = predict_mic()
        success = text != 'background'
    terminate()

############################################ TRACKER ###########################################

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

def init_tracker(frame,tracker_type):
    bbox = cv2.selectROI(frame) 
    tracker = create_tracker_by_name(tracker_type)
    flag = tracker.init(frame, bbox)
    return flag, tracker

def track(frame,tracker,colors):
    flag, bbox = tracker.update(frame)
    if flag == True:
        (x, y, w, h) = [int(v) for v in bbox]
        cv2.rectangle(frame, (x, y), (x + w, y + h), colors, 2)
    return flag

def get_fps(prev_frame_time):
    new_frame_time = time.time()
    fps = round(1/(new_frame_time-prev_frame_time),2)
    prev_frame_time = new_frame_time

    return fps,prev_frame_time

############################################ VIDEO ###########################################

def run_video_command():
    global success,text,ok
    cap = cv2.VideoCapture(0)

    colors = (randint(0, 255), randint(0,255), randint(0, 255))
    tracking = False
    prev_frame_time = 0

    while cap.isOpened():
            flag, frame = cap.read()
            if not flag :
                ok = False
                break

            frame = cv2.resize(frame,dimension)

            fps,prev_frame_time = get_fps(prev_frame_time)

            cv2.putText(frame, f'FPS : {fps}Hz, Command : {text}, Tracking : {tracking}', (2, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)

            if success : 
                if text == 'track' and not tracking : 
                    flag, tracker = init_tracker(frame,tracker_type)
                    if flag:
                        tracking = True
                    else :
                        print('Error while initializing tracker !')
                if text  == 'stop':
                    tracking = False
                if text == 'close':
                    ok = False
                    break


            if tracking : 
                flag = track(frame,tracker,colors)
                if not flag :
                    tracking = False

            cv2.imshow('Frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'): # quit
                ok = False
                break
        
    cap.release()
    cv2.destroyAllWindows()

############################################ Launch ###########################################

videoStreamThread = threading.Thread(target=run_video_command)
videoStreamThread.start()

voiceStreamThread = threading.Thread(target=run_voice_command)
voiceStreamThread.start()
