import pyaudio 
import os

# speech model recognition name
model_name = 'model_1'

# mic recording parameters
FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

 # duration of audio file to detect
duration = 1

# images parameters
dimension = (640,640)

# tracker parameters
tracker_type = 'CSRT'

# object detection 
img_size = 416
score_threshold = 0.2
nms_threshold = 0.4
confidence_threshold = 0.4


folder_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
weigths_path = os.path.join(folder_path,'data','models','best.onnx')

labels_obj = ['pen','pencil','scissors','eraser']

# list labels
labels = ['background','close','eraser','pen','pencil','scissors','stop']
labels.sort()