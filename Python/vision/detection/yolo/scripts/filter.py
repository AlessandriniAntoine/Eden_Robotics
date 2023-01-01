import os
from glob import glob
import contextlib
import time
import sys

import argparse
from functools import partial

import cv2
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
import PyQt5.QtCore as QtCore

class Window(QMainWindow): 
    def __init__(self,folder, images,labels):                                         # x <-- 3
        super().__init__()

        self.folder = folder
        self.images = images
        self.labels = labels
        self.num_file = len(images)
        self.index = 0

        # define widgets
        self.centralwidget = QWidget()
        self.setCentralWidget(self.centralwidget)
        self.lay = QVBoxLayout(self.centralwidget)

        # init display widget
        self.label = QLabel(self)		
        pixmap = QPixmap(self.images[self.index])
        pixmap = pixmap.scaled(QtCore.QSize(320, 320))
        self.label.setPixmap(pixmap)
        self.lay.addWidget(self.label)


        for label in labels:                                  
            btn = QPushButton(label, self)
            btn.clicked.connect(partial(self.click_label, label))
            self.lay.addWidget(btn)


        delete_button = QPushButton('Delete', self)
        delete_button.clicked.connect(self.click_delete)
        self.lay.addWidget(delete_button)

        pass_button = QPushButton('Pass', self)
        pass_button.clicked.connect(self.click_pass)
        self.lay.addWidget(pass_button)

        self.lay.addStretch(1)

        self.show()

    def click_label(self,label):
        if self.index < self.num_file: 
            # rename image
            name = os.path.join(self.folder,f'{label}_{os.path.basename(self.images[self.index])}')
            os.rename(self.images[self.index],name)
            with contextlib.suppress(Exception):
                os.rename(f'{os.path.splitext(self.images[self.index])[0]}.txt',f'{os.path.splitext(name)[0]}.txt')

            # increase index
            self.index += 1
            with contextlib.suppress(Exception):
                pixmap = QPixmap(self.images[self.index])
                pixmap = pixmap.scaled(QtCore.QSize(320, 320))
                self.label.setPixmap(pixmap)

        if self.index >= self.num_file:
            self.close()

    def click_delete(self):
        if self.index < self.num_file: 
            # remove image
            os.remove(self.images[self.index])
            with contextlib.suppress(Exception):
                os.remove(f'{os.path.splitext(self.images[self.index])[0]}.txt')
                
            # increase index
            self.index += 1
            with contextlib.suppress(Exception):
                pixmap = QPixmap(self.images[self.index])
                pixmap = pixmap.scaled(QtCore.QSize(320, 320))
                self.label.setPixmap(pixmap)

        if self.index >= self.num_file:
            self.close()

    def click_pass(self):
        if self.index < self.num_file : 
            # increase index
            self.index += 1
            with contextlib.suppress(Exception):
                pixmap = QPixmap(os.path.join(self.folder,self.images[self.index]))
                pixmap = pixmap.scaled(QtCore.QSize(320, 320))
                self.label.setPixmap(pixmap)

        if self.index >= self.num_file:
            self.close()
############################################ Define ArgParse ###########################################

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--images", help="Path to folder that contains the images.", type=str, nargs="?")
parser.add_argument("-l", "--labels", help="Path to the text file containing the labels", type=str, nargs="?")

args = parser.parse_args()

folder = args.images or ''
labels_path = args.labels or ''

############################################ Check Path Existance ###########################################

flag = True
if not os.path.exists(folder) :
    print('Invalide images path')
    flag = False
else :
    images = [image for image in glob(f'{folder}/*') if image.endswith('.jpg') ]

if not os.path.exists(labels_path) :
    print('Invalide labels path')
    flag = False
else :
    with open(labels_path,'r') as f:
        labels = f.read()
        labels = labels.split('\n')
        if not labels[-1]:
            labels = labels[:-1]
        labels.append('background')
        labels.append('multiple')

############################################ Filter ###########################################


if flag and images:

    # create pyqt5 app
    App = QApplication(sys.argv)

    # create the instance of our Window
    window = Window(folder,images,labels)

    # start the app
    sys.exit(App.exec())

else :
    print('No images')
