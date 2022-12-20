"""
PyQt application to filter the image downloaded
You can save image as background or tomato or delete it
"""

from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
import PyQt5.QtCore as QtCore
import sys
import argparse
import os
import shutil

class Window(QWidget):

    def __init__(self,images,images_folder,save_folder):
        super().__init__()
        
        self.images = images
        self.num_img = len(self.images)
        self.index = 0
        self.folder = images_folder
        self.save_folder = save_folder

        self.acceptDrops()
        self.setWindowTitle("Image")
        self.setGeometry(0, 0, 400, 400)

        # creating label
        self.label = QLabel(self)		
        pixmap = QPixmap(os.path.join(self.folder,self.images[self.index]))
        pixmap = pixmap.scaled(QtCore.QSize(320, 320))
        self.label.setPixmap(pixmap)
        self.label.move(40,60)
        self.label.resize(pixmap.width(),pixmap.height())

        # create delete button
        delete_button = QPushButton("Delete",self)
        delete_button.clicked.connect(self.delete_clicked)
        delete_button.move(16,10)

        # create delete button
        background_button = QPushButton("Background",self)
        background_button.clicked.connect(self.background_clicked)
        background_button.move(112,10)

        # create delete button
        tomato_button = QPushButton("Tomato",self)
        tomato_button.clicked.connect(self.tomato_clicked)
        tomato_button.move(208,10)

        # create delete button
        pass_button = QPushButton("Pass",self)
        pass_button.clicked.connect(self.pass_clicked)
        pass_button.move(304,10)
      
        # show all the widgets
        self.show()

    def delete_clicked(self):
        print("Delete Image")
        if self.index < self.num_img-1 : 
            # remove image
            os.remove(os.path.join(self.folder,self.images[self.index]))

            # increase index
            self.index += 1

            # change image
            pixmap = QPixmap(os.path.join(self.folder,self.images[self.index]))
            pixmap = pixmap.scaled(QtCore.QSize(320, 320))
            self.label.setPixmap(pixmap)
            self.label.move(40,60)
            self.label.resize(pixmap.width(),pixmap.height())
        
        if self.index == self.num_img-1:
            # remove image
            os.remove(self.images[self.index])
            self.index +=1
            print('No more image')

        if self.index >= self.num_img:
            print('No more image')

    def background_clicked(self):
        print("Image save as background")   
        if self.index < self.num_img-1 : 
            # move image
            src_path = os.path.join(self.folder,self.images[self.index])
            dst_path = os.path.join(self.save_folder,f'background.{self.images[self.index]}')
            shutil.move(src_path, dst_path)

            # increase index
            self.index += 1

            # change image
            pixmap = QPixmap(os.path.join(self.folder,self.images[self.index]))
            pixmap = pixmap.scaled(QtCore.QSize(320, 320))
            self.label.setPixmap(pixmap)
            self.label.move(40,60)
            self.label.resize(pixmap.width(),pixmap.height())
        
        if self.index == self.num_img-1:
            # move image
            src_path = os.path.join(self.folder,self.images[self.index])
            dst_path = os.path.join(self.save_folder,f'background.{self.images[self.index]}')
            shutil.move(src_path, dst_path)
            
            self.index += 1
            print('No more image')

        if self.index >= self.num_img:
            print('No more image')

    def tomato_clicked(self):
        print("Image save as tomato") 
        if self.index < self.num_img-1 : 
            # move image
            src_path = os.path.join(self.folder,self.images[self.index])
            dst_path = os.path.join(self.save_folder,f'tomato.{self.images[self.index]}')
            shutil.move(src_path, dst_path)

            # increase index
            self.index += 1

            # change image
            pixmap = QPixmap(os.path.join(self.folder,self.images[self.index]))
            pixmap = pixmap.scaled(QtCore.QSize(320, 320))
            self.label.setPixmap(pixmap)
            self.label.move(40,60)
            self.label.resize(pixmap.width(),pixmap.height())
        
        if self.index == self.num_img-1:
            # move image
            src_path = os.path.join(self.folder,self.images[self.index])
            dst_path = os.path.join(self.save_folder,f'tomato.{self.images[self.index]}')
            shutil.move(src_path, dst_path)
            self.index += 1
            print('No more image')

        if self.index >= self.num_img:
            print('No more image')

    def pass_clicked(self):
        print("Pass without deleting") 
        if self.index < self.num_img-1 : 
            # increase index
            self.index += 1

            # change image
            pixmap = QPixmap(os.path.join(self.folder,self.images[self.index]))
            pixmap = pixmap.scaled(QtCore.QSize(320, 320))
            self.label.setPixmap(pixmap)
            self.label.move(40,60)
            self.label.resize(pixmap.width(),pixmap.height())
        
        if self.index >= self.num_img-1:
            # move image
            self.index += 1
            print('No more image')


parser = argparse.ArgumentParser()
parser.add_argument("-i", "--images", help="Path to folder that contains the images.", type=str, nargs="?")
parser.add_argument("-s", "--save", help="Path to folder to save the images.", type=str, nargs="?")
args = parser.parse_args()

images_folder = args.images or ''
save_folder = args.save or ''

flag = True
if not os.path.exists(images_folder) :
    print('Invalide images path')
    flag = False
if not os.path.exists(save_folder) :
    print('Invalide saving path')
    flag = False


if flag : 
    images = os.listdir(images_folder)
    if not images : 
        print('No images')

    else: 
        # create pyqt5 app
        App = QApplication(sys.argv)

        # create the instance of our Window
        window = Window(images,folder,save_folder)

        # start the app
        sys.exit(App.exec())
