from PyQt5.QtWidgets import *
import sys
import argparse
import contextlib
import os
from glob import glob
from pydub import AudioSegment
from pydub.playback import play
import random
import shutil

def get_label(name,labels):
    for label in labels:
        label = f'{label}_'
        if label in name:
            return label[:-1]

class Window(QWidget):

    def __init__(self,files_path,labels,test_path,train_path,test_size):
        super().__init__()
                
        self.files_path = files_path
        self.test_path = test_path
        self.train_path = train_path
        self.num_file = len(files_path)

        self.index = 0
        self.keep = []
        
        self.test_size = test_size
        self.labels = labels

        label = get_label(self.files_path[self.index],self.labels)
        self.label_display = QLabel(f'Label : {label}',self)
        self.label_display.move(50,65)

        file = self.files_path[self.index].split('/')[-1]
        self.file_display = QLabel(f'{file}',self)
        self.file_display.move(2,80)

        self.acceptDrops()
        self.setWindowTitle("Image")
        self.setGeometry(0, 0, 215, 100)

        # create delete button
        delete_button = QPushButton("Delete",self)
        delete_button.clicked.connect(self.delete_clicked)
        delete_button.setGeometry(5,5,100,25)

        # create delete button
        keep_button = QPushButton("Keep",self)
        keep_button.clicked.connect(self.keep_clicked)
        keep_button.setGeometry(110,5,100,25)

        # create play button
        play_button = QPushButton("Play",self)
        play_button.clicked.connect(self.play_clicked)
        play_button.setGeometry(110,35,100,25)

        # create split button (only if test and train paths has been specify)
        if self.test_path and self.train_path:
            split_button = QPushButton("Split",self)
            split_button.clicked.connect(self.split_clicked)
            split_button.setGeometry(5,35,100,25)

        # show all the widgets
        self.show()

    def delete_clicked(self):
        if self.index < self.num_file : 
            # remove image
            os.remove(self.files_path[self.index])

            # increase index
            self.index += 1
            with contextlib.suppress(Exception):
                label = get_label(self.files_path[self.index],self.labels)
                self.label_display.setText(f'Label : {label}')
                
                file = self.files_path[self.index].split('/')[-1]
                self.file_display.setText(f'{file}')

                sound = AudioSegment.from_file(self.files_path[self.index], 'wma')
                play(sound)

        if self.index >= self.num_file:
            self.file_display.setText('No audio')
            if not self.test_path and not self.train_path:
                print('No more audio')
                self.close()

    def keep_clicked(self):
        if self.index < self.num_file :
            self.keep.append(self.files_path[self.index])
            self.index += 1
            with contextlib.suppress(Exception):
                label = get_label(self.files_path[self.index],self.labels)
                self.label_display.setText(f'Label : {label}')

                file = self.files_path[self.index].split('/')[-1]
                self.file_display.setText(f'{file}')

                sound = AudioSegment.from_file(self.files_path[self.index], 'wma')
                play(sound)

        if self.index >= self.num_file:
            self.file_display.setText('No audio')
            self.file_display.move(60,80)
            if not self.test_path and not self.train_path:
                print('No more audio')
                self.close()

    def play_clicked(self):
        if self.index >= self.num_file:
            self.file_display.setText('No audio')
            self.file_display.move(60,80)
        else :
            sound = AudioSegment.from_file(self.files_path[self.index], 'wma')
            play(sound)

    def split_clicked(self):
        test =  set(random.sample(self.keep, int(self.test_size*len(self.keep))))
        train = set(self.keep) - test

        for file_path in list(train) :
            src_path = file_path
            file_name = os.path.basename(file_path)
            for label in self.labels:
                label = f'{label}_'
                if label in file_name:
                    dst_path = os.path.join(self.train_path,label,file_name)
                    shutil.move(src_path, dst_path)
                    break
        
        for file_path in list(test) :
            src_path = file_path
            file_name = file_path.split('/')[-1]
            for label in self.labels:
                label = f'{label}_'
                if label in file_name:
                    dst_path = os.path.join(self.test_path,label,file_name)
                    shutil.move(src_path, dst_path)
                    break
        self.close()

############################################ Define ArgParse ###########################################

parser = argparse.ArgumentParser()
parser.add_argument("-a", "--audio", help="Path to folder that contains the audio.", type=str, nargs="?")
parser.add_argument("-e", "--test", help="Path to folder to put the test files.", type=str, nargs="?")
parser.add_argument("-r", "--train", help="Path to folder  to put the train files.", type=str, nargs="?")
parser.add_argument("-t", "--test_size", help="Test size to split", type=float, nargs="?")

args = parser.parse_args()

audio_path = args.audio or ''
test_path = args.test or ''
train_path = args.train or ''
test_size = args.test_size or 0

############################################ Check Path Existance ###########################################

flag = True
if not os.path.exists(audio_path) :
    print('Invalide audio path')
    flag = False

if (test_path and not train_path) or (not test_path and train_path):
    print('Test and Train path need to be specify together.')
    flag = False

if test_path and train_path:
    if not os.path.exists(test_path) :
        print('Test Folder does not exist')
        flag = False

    if not os.path.exists(train_path) :
        print('Train Folder does not exist')
        flag = False

############################################ Filter ###########################################

if flag : 
    labels = os.listdir(audio_path)
    labels.sort()
    try : 
        list_files = glob(f"{audio_path}/**/*", recursive=True)
        for label in labels:
            list_files.remove(os.path.join(audio_path,label))
        if list_files:
            # create pyqt5 app
            App = QApplication(sys.argv)

            # create the instance of our Window
            window = Window(list_files,labels,test_path,train_path,test_size)

            # start the app
            sys.exit(App.exec())

        else :
            print('No audio')
    except Exception : 
        print('Error while loading files')