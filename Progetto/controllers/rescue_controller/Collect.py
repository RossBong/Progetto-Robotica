from Movement import Movement
from Cam import Cam
import numpy as np
from controller import Robot, Motor, Camera, CameraRecognitionObject
import time
class Collect:
    
    def __init__(self,movement,cam,robot,pos_start):
        self.movement=movement
        self.cam=cam
        self.map=[]
        self.robot=robot
        self.pos_start=pos_start
        self.lift_motor=robot.getDevice("lift motor")
        self.finger_motor=robot.getDevice("finger motor::left")
        self.gripperMaxSpeed=0.1
        
    def start_collect(self):
        print("Inizio a raccogliere gli oggetti")
        objs_coord=np.argwhere(self.map == 3)
        for obj in objs_coord:
            
            path=self.movement.find_path_obj(self.map,obj[0],obj[1])
            self.movement.follow_path(path)
            self.movement.obj_dir(obj[0],obj[1])
            ogg, _=self.cam.recognition()
            self.aggancia()
            path_reverse=self.movement.find_path_obj(self.map,self.pos_start[0],self.pos_start[1])
            self.movement.follow_path(path_reverse)
            self.rilascia()
            print(f"Ho consegnato un {ogg}")
            
             
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
        self.lift(0.02)
        self.move_fingers(0.1)
        self.movement.move(dist)
        self.move_fingers((size-0.04)/2)
        self.lift(-0.03)

        self.movement.move_back(dist)
 
        
        
    def rilascia(self):
        
        self.movement.move(0.5)
        self.lift(0.02)
        self.move_fingers(0.1)
        self.movement.move_back(0.5)
        self.move_fingers(0)
       
        
        
          
          
          
          
          
          
          
          
          
          