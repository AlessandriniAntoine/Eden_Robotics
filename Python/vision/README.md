# Vision

During this project we used openCV librairy.

First, we display the output of a webcam.

Then we set up an object recognition coupled with an object tracking. Thanks to this, the camera recognizes the object, targets it and tracks it. Once the object is targeted, the tracking is done only on this object, regardless of the presence of similar objects in the field of view.

## Detection

In order to get ride of the user, a object detector can be used. The [detection](./detection/) folder shows two possible algorithms :

- haar cascade
- yolo5

Everything is show to create a virtual environment and create our own model from custom images. It is easier to create a virtual environment because different version of the same package are needed. Once the model is created, you can use it with any version of opencv.

## Tracking

Once the tomato has been found, we want to track it. The [tracking](./tracking/) folder shows how to use different algorithms available in opencv. One personal application is shown in the [main.py](./tracking/main.py) file.

For this you need the following packages :

- opencv-python
- opencv-contrib-python

## Useful Links

- [openCV_documentation](https://docs.opencv.org/4.x/)
- [openCV_tutorial](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [computer_vision_projects](https://www.computervision.zone/)
