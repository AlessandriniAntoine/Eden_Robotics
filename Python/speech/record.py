import pyaudio
import time
import wave
import os
import argparse

# save audio
def record(label):
    
    duration = 1 #s

    p = pyaudio.PyAudio()
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=FRAMES_PER_BUFFER
    )

    frames = []
    for _ in range(int(RATE / FRAMES_PER_BUFFER * duration)):
        data = stream.read(FRAMES_PER_BUFFER)
        frames.append(data)

    stream.stop_stream()
    stream.close()
    p.terminate()

    path = os.path.join(files_path,label,f'{label}_{time.strftime("%Y%m%d_%H%M%S")}.wav')
    wf = wave.open(path, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames)) # write as binary string
    wf.close()

############################# Set Recording Parameters #############################

FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000


############################# Read Arguments #############################

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--num", help="Number of files per label.", type=int, nargs="?")
parser.add_argument("-l", "--labels", help="List of labels to record", type=str, nargs="+")
args = parser.parse_args()

num_per_label = args.num or 0
if num_per_label == 0 : 
    print('Please precise number of files per label to record.')

############################# Get Labels #############################

files_path = os.path.join(os.path.dirname(__file__),'data','audio','files')    
labels = args.labels or os.listdir(files_path)

flag=True
for label in labels : 
    if not os.path.exists(os.path.join(files_path,label)):
        print(f'Label forlder {label} does not exists')
        flag = False
############################# Record #############################

# record data
if flag : 
    for label in labels:
        print(f'Record for {label}')
        time.sleep(1.5)
        for i in range(num_per_label):
            print(f'Start recording number {i}')
            record(label)
            time.sleep(0.5)