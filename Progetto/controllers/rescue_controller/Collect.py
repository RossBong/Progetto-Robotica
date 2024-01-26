from Movement import Movement
from Cam import Cam
import numpy as np
from TTS import TTS
from controller import Robot, Motor, Camera, CameraRecognitionObject
import time
class Collect:
    
    def __init__(self,movement,cam,robot,pos_start, tts):
        self.movement=movement
        self.cam=cam
        self.map=[]
        self.robot=robot
        self.pos_start=pos_start
        self.lift_motor=robot.getDevice("lift motor")
        self.finger_motor=robot.getDevice("finger motor::left")
        self.gripperMaxSpeed=0.1
        self.tts=tts
        
        
    def start_collect(self):
        txt="Inizio a raccogliere gli oggetti"
        print(txt)
        self.tts.text_to_speech(txt)
        objs_coord=np.argwhere(self.map == 3)
        for obj in objs_coord:
            
            path=self.movement.find_path_obj(self.map,obj[0],obj[1])
            self.movement.follow_path(path)
            self.movement.obj_dir(obj[0],obj[1])
            ogg, _=self.cam.recognition()
            self.aggancia()
            path_reverse=self.movement.find_path_obj(self.map,self.pos_start[0],self.pos_start[1])
            self.movement.follow_path(path_reverse)
            self.rilascia(ogg)
            
             
    def lift(self,position):
         end_time = 1 + self.robot.getTime()
         self.lift_motor.setPosition(position)
         while self.robot.step(32) != -1:
             if self.robot.getTime() < end_time:
                 self.lift_motor.setVelocity(self.gripperMaxSpeed)
             else:
                break
           
        
    def move_fingers(self,position):
        end_time = 1 + self.robot.getTime()
        self.finger_motor.setPosition(position)
        while self.robot.step(32) != -1:
            if self.robot.getTime() < end_time:
                self.finger_motor.setVelocity(self.gripperMaxSpeed)
            else:
                break
           
    
    def aggancia(self):
        self.movement.lidarsensor()
        dist=self.movement.lidar_value[0]-0.05
        objs = self.cam.camera.getRecognitionObjects()
        size=objs[0].getSize()[0]
        self.get_well(objs)
        self.lift(0.02)
        self.move_fingers(0.1)
        self.movement.move(dist-0.02)
        self.move_fingers((size-0.05)/2)
        self.lift(-0.04)
        self.movement.move_back(dist)
        
    def get_well(self, objs):
    
        direction_now=self.movement.direction()
        aggiustamento=objs[0].getPosition()[1]
        if objs[0].getPosition()[1]<-0.03:
            if direction_now=="West":
                self.movement.rotate("North")
            elif direction_now=="South":
                self.movement.rotate("West")
            elif direction_now=="Nord":
                self.movement.rotate("East")
            elif direction_now=="East":
                self.movement.rotate("South")
            self.movement.move(abs(aggiustamento))
            self.movement.rotate(direction_now)
        elif objs[0].getPosition()[1]>0.03:
            if(direction_now=="West"):
                self.movement.rotate("South")
            elif direction_now=="South":
                self.movement.rotate("East")
            elif direction_now=="Nord":
                self.movement.rotate("West")
            elif direction_now=="East":
                self.movement.rotate("North")
            self.movement.move(aggiustamento)
            self.movement.rotate(direction_now)
        
        
    def rilascia(self, ogg):
        
        self.movement.move(0.5)
        self.lift(0.02)
        self.move_fingers(0.1)
        if ogg=="box_foto":
            txt=f"Ho consegnato un pacco contenente foto"
        elif ogg=="box_gioielli":
            txt=f"Ho consegnato un un pacco contenente gioielli"
        elif ogg=="box_soldi" or ogg=="box_soldi2":
            txt=f"Ho consegnato un pacco contenente soldi"
        print(txt)
        self.tts.text_to_speech(txt)
        self.movement.move_back(0.5)
        self.move_fingers(0)