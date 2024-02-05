from controller import  Camera, CameraRecognitionObject
import numpy as np

class Cam:

    def __init__(self,robot,timestep):
        self.camera = robot.getDevice('camera')
        self.camera.enable(timestep)
        self.camera.recognitionEnable(timestep)
        
       
    def recognition(self):
        objs = self.camera.getRecognitionObjects()#riconoscimento oggetti
        if(len(objs)>0):
            min=100
            for i in range(len(objs)):
                #consideriamo esclusivamente l'oggetto più vicino
                if objs[i].getPosition()[0]<min:
                    min=objs[i].getPosition()[0]
                    dist_o=objs[i].getPosition()[1]#distaza lungo l'asse orizzontale dal centro della camera 
                    index=i
            color= objs[index].getColors()
            
            if( dist_o>-0.3 and  dist_o<0.3):#riconoscumento oggetto frontale 
            
                #in base al colore di recognition restituiamo in output il tipo di oggetto e la distanza da esso
                if(color[0]==1.0 and color[1]==1.0 and color[2]==0.0):
                    return"box_gioielli", min 
                elif(color[0]==1.0 and color[1]==0.0 and color[2]==0.0):
                    return"umano",min
                elif(color[0]==0.0 and color[1]==1.0 and color[2]==0.5):
                    return"box_soldi",min
                elif(color[0]==0.0 and color[1]==1.0 and color[2]==0.0):
                    return"box_foto",min
        return " ",10        
   
        