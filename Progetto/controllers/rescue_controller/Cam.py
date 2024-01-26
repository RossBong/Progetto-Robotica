from controller import  Camera, CameraRecognitionObject, LightSensor
import numpy as np
import argparse
 
class Cam:

    def __init__(self,robot,timestep):
        self.camera = robot.getDevice('camera')
        self.camera.enable(timestep)
        self.camera.recognitionEnable(timestep)
        #self.lumos=robot.getDevice('light sensor')
        #self.lumos.enable(timestep)
       
        #self.spotlight = robot.getDevice('light pioneer')
       
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
    """    
    def sensorLight(self):
    
        if(self.lumos.getValue()<50):
            self.spotlight.enable(32)
        elif(self.lumos.getValue()<50):
            self.spotlight.disable(32)
      """      
        