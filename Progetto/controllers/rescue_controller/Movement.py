import math
import numpy as np
from controller import Robot, DistanceSensor, Motor, Lidar
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from Particle_filter import Particle_filter

class Movement:
    
    def __init__(self,robot,timestep,x_start,y_start,PF):
        self.robot=robot
        self.timestep=timestep
        self.leftMotor = robot.getDevice('left wheel')
        self.rightMotor = robot.getDevice('right wheel')
        
    
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
    
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
    
        #position sensor instance
        self.left_ps= robot.getDevice('left wheel sensor')
        self.left_ps.enable(timestep)
        self.right_ps= robot.getDevice('right wheel sensor')
        self.right_ps.enable(timestep)
       
        #lidar
        self.lidar_front= robot.getDevice('lidar_front')
        self.lidar_front.enable(timestep)
        self.lidar_back= robot.getDevice('lidar_back')
        self.lidar_back.enable(timestep)
        self.lidar_dx= robot.getDevice('lidar_dx')
        self.lidar_dx.enable(timestep)
        self.lidar_sx= robot.getDevice('lidar_sx')
        self.lidar_sx.enable(timestep)
        
        #enable imu
        self.imu = robot.getDevice('inertial unit')
        self.imu.enable(timestep)
        self.MAX_SPEED = 4
    
        self.ps_values=[0, 0] #radianti relativi all'odometria
        self.dist_values=[0,0] #distanza percorsa misurata con l'odometria
        
        self.raggio_ruota=0.195/2
        
        self.dist_ruote=0.34215
        self.d_mid=self.dist_ruote/2     
        
        self.circonf_ruota=self.raggio_ruota*2*math.pi #0.61 m
        self.enc_unit=self.circonf_ruota/6.29 #porzione di circonferenza per radiante
        
        self.robot_pose=[x_start,y_start,"North"]
        
        self.lidar_value=[[],[],[],[]]
        
        self.PF=PF
        
        
        

    def lidarsensor(self):
       self.lidar_value[0]=min(self.lidar_front.getRangeImage())
       self.lidar_value[1]=min(self.lidar_back.getRangeImage())
       self.lidar_value[2]=min(self.lidar_dx.getRangeImage())
       self.lidar_value[3]=min(self.lidar_sx.getRangeImage())
       
    def odo(self):
   
       self.ps_values[0]=self.left_ps.getValue()
       self.ps_values[1]=self.right_ps.getValue()
       
       self.dist_values[0]=self.ps_values[0]*self.enc_unit
       self.dist_values[1]=self.ps_values[1]*self.enc_unit
       
       
    def direction(self):
        q=(self.imu.getRollPitchYaw()[2] * 180) / 3.14159
        if (q<= -135 and q >= -180) or (135 <= q <= 180):
            return "West"
        elif q <= -45 and q > -135:
            return "South"
        elif 45 <= q <= 135:
            return "North"
        elif (-45 < q <= 0) or (0 <= q < 45):
            return "East" 
        
    def robot_update(self,dist):
 
        dir=self.direction()
        if (dist==0):  #il robot ha ruotato
        
            self.robot_pose[2]=dir
        else:  #il robot si muove di una casella nella precedente direzione
            if(dir=="North"):
                self.robot_pose[0]-=dist
            elif(dir=="West"):
                self.robot_pose[1]-=dist
            elif(dir=="East"):
                self.robot_pose[1]+= dist
            elif(dir=="South"):
                self.robot_pose[0]+=dist
                
    def lidar_permutation(self,dir):
    
        self.lidarsensor()
        sd=self.lidar_value
        #self.movement.lidarsensor()
        #sd=self.movement.lidar_value #North,South,East,West 
        
        #dir =self.robot_pose[2]
        
        if(dir=="North"):
            return sd
        elif(dir=="East"):
            sd_p=[sd[3],sd[2],sd[0],sd[1]]
            return sd_p
        elif(dir=="South"):
            sd_p=[sd[1],sd[0],sd[3],sd[2]]
            return sd_p
        elif(dir=="West"):
            sd_p=[sd[2],sd[3],sd[1],sd[0]]
            return sd_p    
    
    def layer_reattivo(self,delta):
        
           
           self.lidarsensor()
           
           if(self.lidar_value[0]<delta):
               
               return False
               
           else:
               return True 
    
        
    def stop_motors(self):
        
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        
    def move(self,dist):
        maxspeed=self.MAX_SPEED
       
        self.odo()
        start_dist=self.dist_values.copy()
        
        while self.robot.step(self.timestep) != -1:
            
            self.odo()
            layer_reattivo=self.layer_reattivo(0.3)
            if self.dist_values[0]-start_dist[0]<=dist  and layer_reattivo:
                
                self.leftMotor.setVelocity(maxspeed)
                self.rightMotor.setVelocity(maxspeed)
                
            else:
                self.stop_motors()
                break
        
        self.robot_update(round(self.dist_values[0]-start_dist[0]))
        
        
        
        
    def move_back(self,dist):
        maxspeed=-self.MAX_SPEED

        self.odo()
        start_dist=self.dist_values.copy()
        
        while self.robot.step(self.timestep) != -1:
            self.odo()
            if start_dist[0]-self.dist_values[0]<=dist :
                
                self.leftMotor.setVelocity(maxspeed)
                self.rightMotor.setVelocity(maxspeed)
                
            else:
                self.stop_motors()
                break
        
        self.robot_update(-round(start_dist[0]-self.dist_values[0]))   
    
        
    def rotate(self,dir):
        last_dir=self.direction()
        left_speed=0.5
        right_speed=-0.5
        degree=999
        if(dir=="North" and last_dir!="North"):
            if(last_dir=="East"):
                left_speed=-0.5
                right_speed=0.5
            degree=90
        elif(dir=="South"and last_dir!="South"):
            if(last_dir=="West"):
                    left_speed=-0.5
                    right_speed=0.5
            degree=-90
        elif(dir=="East" and last_dir!="East"):
            if(last_dir=="South"):
                    left_speed=-0.5
                    right_speed=0.5
            degree=0
        elif(dir=="West" and last_dir!="West"):
            if(last_dir=="North"):
                    left_speed=-0.5
                    right_speed=0.5
            degree=180
        
       
        while self.robot.step(self.timestep) != -1:
            
            new_degree=round((self.imu.getRollPitchYaw()[2] * 180) / 3.14159)
            if(degree==999):
                break
            if(degree!=new_degree):
                self.leftMotor.setVelocity(left_speed)
                self.rightMotor.setVelocity(right_speed)
            else:
                self.stop_motors()
                break
             
        self.robot_update(0)#aggiornamernto direzione
        
    
    def follow_path(self,path):
        layer_reattivo=True
        for x,y in path[1:]:
          if((x-self.robot_pose[0])==0 and y>self.robot_pose[1] ):
              if(self.robot_pose[2]!="East"):
                  self.rotate("East")
              if(self.layer_reattivo(1)==False):
                  return False    
              self.move(1)
          elif((x-self.robot_pose[0])==0 and y<self.robot_pose[1] ):
              if(self.robot_pose[2]!="West"):
                  self.rotate("West")
              if(self.layer_reattivo(1)==False):
                  return False    
              self.move(1)
          elif((y-self.robot_pose[1])==0 and x<self.robot_pose[0] ):
              if(self.robot_pose[2]!="North"):
                  self.rotate("North")
              if(self.layer_reattivo(1)==False):
                  return False    
              self.move(1)
          elif((y-self.robot_pose[1])==0 and x>self.robot_pose[0] ):
              if(self.robot_pose[2]!="South"):
                  self.rotate("South")
              if(self.layer_reattivo(1)==False):
                  return False    
              self.move(1)
          
        return True

              
    def obj_dir(self,x,y):
         x_robot=self.robot_pose[0]
         y_robot=self.robot_pose[1]
         if(x<x_robot):
             self.rotate("North")
         elif(x>x_robot):
             self.rotate("South")
         elif(y<y_robot):
             self.rotate("West")     
         elif(y>y_robot):
             self.rotate("East")
             
             
    def obj_front_pose(self):
        x_robot=self.robot_pose[0]
        y_robot=self.robot_pose[1]
        dir=self.robot_pose[2]
        if(dir=="North"):
            return x_robot+1,y_robot
        elif(dir=="South"):
            return x_robot-1,y_robot
        elif(dir=="East"):
            return x_robot,y_robot+1
        elif(dir=="West"):
            return x_robot,y_robot-1   
                 
    def find_path_obj(self,map,x,y):
       
        x_robot=self.robot_pose[0]
        y_robot=self.robot_pose[1]
        
        map=map.copy()
        map[x,y]=0
        grid = Grid(matrix=~abs(map).astype(bool))
        start = grid.node(y_robot,x_robot)
        end = grid.node(y,x)
        
        # Usa l'algoritmo A* per trovare il percorso
        finder = AStarFinder()
        path, runs = finder.find_path(start, end, grid)
        path = [(nodo.y, nodo.x) for nodo in path]
        
        #grid.cleanup()
        return path[:-1]
    
    def get_opposite_dir(self, dir):
        if(dir=="North"):
            return"South"
        elif(dir=="South"):
            return"North"
        elif(dir=="East"):
            return "West"
        elif(dir=="West"):
            return"East"
    
    def random_movement(self):
        count=0
        pos_est=[]
        last_dir=self.direction()
        rb_list=[self.robot_pose[:2]]
        while(len(rb_list)<6):#il robot esce dal while quando avrà visitato 5 celle diverse
            
            print(rb_list)
            
            sd=self.lidar_permutation(self.direction())
            
               
            if(sd[0]>1 and ((self.robot_pose[:2]+[-1,0]) not in rb_list)):
                self.rotate("North")
                self.move(1)
                pos_est=self.PF.particle_filter([-1,0],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
                
            elif(sd[2]>1 and ((self.robot_pose[:2]+[0,1]) not in rb_list)):
                self.rotate("East")
                self.move(1)
                pos_est=self.PF.particle_filter([0,1],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
                
            elif(sd[3]>1 and ((self.robot_pose[:2]+[0,-1]) not in rb_list)):
                self.rotate("West")
                
                self.move(1)
                pos_est=self.PF.particle_filter([0,-1],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
                
            elif(sd[1]>1  and ((self.robot_pose[:2]+[1,0]) not in rb_list)):
            
                self.rotate("South")
                self.move(1)
                pos_est=self.PF.particle_filter([1,0],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
            
            else:
                self.rotate(self.get_opposite_dir(last_dir))
                self.move(1)
                
            last_dir=self.direction()
           
        return pos_est    
    
    def agg_filtro(self):
          print("aggiornamento filtro ")
          # ridistribuzione particelle
          self.PF.redistribution()
          # movimenti casuali di almeno 3 celle al fine di ottenere la posizione
          pos_est=self.random_movement()
          
          self.robot_pose[:2]=pos_est
          self.robot_pose[2]=self.direction()
                  
                        
    def follow_path_filtered(self,path,map):
        layer_reattivo=True
        
       
        self.PF.map=map
        pos_est=[]
        count_cells=0
        for x,y in path[1:]:
          self.lidarsensor()
          sd=self.lidar_value
          sd_p=self.lidar_permutation(self.direction())
          if(count_cells >5): # controllo ogni 5 celle se le ruote hanno slittato
              
              if(self.robot_pose[:2] in self.PF.position_estimate(sd_p,self.direction())):
                  
                  print("uguali")
              else:
                  #slittamento
              
                  
                  self.agg_filtro()
                  return False  
                  
             
              
                
          if((x-self.robot_pose[0])==0 and y>self.robot_pose[1] ):
              if(self.robot_pose[2]!="East"):
                  self.rotate("East")
              if(self.layer_reattivo(0.5)==False):
                  
                  
                  self.agg_filtro()
                  return False  
                     
              self.move(1)
              count_cells+=1
              
              sd_p=self.lidar_permutation(self.direction())
              pos_est=self.PF.particle_filter([0,1],sd_p,self.direction())
             
          elif((x-self.robot_pose[0])==0 and y<self.robot_pose[1] ):
              if(self.robot_pose[2]!="West"):
                  self.rotate("West")
              if(self.layer_reattivo(0.5)==False):
                  self.agg_filtro()
                  return False  
              self.move(1)
              count_cells+=1
              sd_p=self.lidar_permutation(self.direction())
              pos_est=self.PF.particle_filter([0,-1],sd_p,self.direction())
             
          elif((y-self.robot_pose[1])==0 and x<self.robot_pose[0] ):
              if(self.robot_pose[2]!="North"):
                  self.rotate("North")
              if(self.layer_reattivo(0.5)==False):
                  self.agg_filtro()
                  return False     
              self.move(1)
              count_cells+=1
              sd_p=self.lidar_permutation(self.direction())
              pos_est=self.PF.particle_filter([-1,0],sd_p,self.direction())
              
          elif((y-self.robot_pose[1])==0 and x>self.robot_pose[0] ):
              if(self.robot_pose[2]!="South"):
                  self.rotate("South")
              if(self.layer_reattivo(0.5)==False):
                  self.agg_filtro()
                  return False   
              self.move(1)
              count_cells+=1
              sd_p=self.lidar_permutation(self.direction())
              pos_est=self.PF.particle_filter([1,0],sd_p,self.direction())
              
        
              
        return True
