from controller import  Camera, CameraRecognitionObject
import cv2
import numpy as np
import argparse

#from keras.applications.vgg16 import VGG16
#from keras.preprocessing import image
#from keras.applications.vgg16 import preprocess_input, decode_predictions

 
class Cam:

    def __init__(self,robot,timestep):
        self.camera = robot.getDevice('camera')
        self.camera.enable(timestep)
        self.camera.recognitionEnable(timestep)
        #self.model=VGG16(weights='imagenet')
        
    def recognition(self):
        objs = self.camera.getRecognitionObjects()
        if(len(objs)>0):
            min=100
            for i in range(len(objs)):
                if objs[i].getPosition()[0]<min:
                    min=objs[i].getPosition()[0]
                    y=objs[i].getPosition()[1]
                   
                    index=i
            color= objs[index].getColors()
            if(y>-0.3 and y<0.3):#imponiamo di vedere esclusivamente l'oggetto davanti al robot
                if(color[0]==1.0 and color[1]==1.0 and color[2]==0.0):
                    return"box_gioielli", min 
                elif(color[0]==1.0 and color[1]==0.0 and color[2]==0.0):
                    return"umano",min
                elif(color[0]==0.0 and color[1]==1.0 and color[2]==0.5):
                    return"box_soldi",min
                elif(color[0]==0.0 and color[1]==1.0 and color[2]==0.0):
                    return"box_foto",min
        return " ",10        
        
    def VGG(self):

        # Caricamento del modello pre-addestrato VGG16
        
         #f3f3tfw4g34g
        # Caricamento dell'immagine di input
        img=self.camera.getImageArray()
        
        img =np.array(img)
        print(img.shape)
        
        
        x = np.expand_dims(img, axis=0)
        x = preprocess_input(x)
         
        # Predizione dell'oggetto nell'immagine
        preds = self.model.predict(x)
         
        # Stampa delle prime 3 classi predette
        print('Predizioni:', decode_predictions(preds, top=3)[0])