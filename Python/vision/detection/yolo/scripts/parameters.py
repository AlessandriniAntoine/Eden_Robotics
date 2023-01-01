
import os

img_size = 416
score_threshold = 0.2
nms_threshold = 0.4
confidence_threshold = 0.4

model_number = 1

folder_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
models_path = os.path.join(folder_path,'data','models')
weigths_path = os.path.join(models_path,f'model_{model_number}','train','weights','best.onnx')

images_folder = os.path.join(folder_path,'data','test','images')
images = [os.path.join(images_folder,image) for image in os.listdir(images_folder)]