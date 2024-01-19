from controller import  Camera, CameraRecognitionObject
import cv2
import numpy as np
import argparse
from darkflow.net.build import TFNet

class Cam:

    def __init__(self,robot,timestep):
        self.camera = robot.getDevice('camera')
        self.camera.enable(timestep)
        self.camera.recognitionEnable(timestep)
        
    def recognition(self):
        objs = self.camera.getRecognitionObjects()
        
        if len(objs) > 0:
                # get the first object
                obj = objs[0]
                print(obj)
                
    def yoloRec(self):
      
         
        # Definisci le opzioni per YOLO e carica il modello
        options = {"model": "cfg/yolo.cfg", "load": "bin/yolov2.weights", "threshold": 0.1}
        yolo = TFNet(options)
         
        # Carica l'immagine
        img = cv2.imread('images/your_image.jpg', cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
         
        # Usa YOLO per riconoscere gli oggetti nell'immagine
        results = yolo.return_predict(img)
         
        # Stampa i risultati
        for result in results:
            print(result)