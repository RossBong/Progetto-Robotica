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
        self.PF=PF
    
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

        self.circonf_ruota=self.raggio_ruota*2*math.pi #0.61 m
        self.enc_unit=self.circonf_ruota/6.29 #porzione di circonferenza per radiante
        
        self.robot_pose=[x_start,y_start,"North"]#posizione robot
        
        self.lidar_value=[[],[],[],[]]#valori sensori distanza nell'ordine[Nord Sud Est Ovest]
        

    def lidarsensor(self):#aggiornamento valori
       self.lidar_value[0]=min(self.lidar_front.getRangeImage())
       self.lidar_value[1]=min(self.lidar_back.getRangeImage())
       self.lidar_value[2]=min(self.lidar_dx.getRangeImage())
       self.lidar_value[3]=min(self.lidar_sx.getRangeImage())
       
    def odo(self):#aggiornamento valori
       
       #radianti percorsi dalle ruote
       self.ps_values[0]=self.left_ps.getValue()
       self.ps_values[1]=self.right_ps.getValue()
       #conversione radianti in metri
       self.dist_values[0]=self.ps_values[0]*self.enc_unit
       self.dist_values[1]=self.ps_values[1]*self.enc_unit
       
       
    def direction(self):#direzione robot
        q=(self.imu.getRollPitchYaw()[2] * 180) / 3.14159
        if (q<= -135 and q >= -180) or (135 <= q <= 180):
            return "West"
        elif q <= -45 and q > -135:
            return "South"
        elif 45 <= q <= 135:
            return "North"
        elif (-45 < q <= 0) or (0 <= q < 45):
            return "East"
            
            
    def obj_dir(self,x,y):#rotazione verso le coordinate x y
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
             
             
    def obj_front_pose(self):#coordinate oggetto di fronte al robot
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
            
    def get_opposite_dir(self, dir):
        if(dir=="North"):
            return"South"
        elif(dir=="South"):
            return"North"
        elif(dir=="East"):
            return "West"
        elif(dir=="West"):
            return"East"
            
                   
    def robot_update(self,dist):#aggiornamento posizione e direzione robot
 
        dir=self.direction()
        if (dist==0):  #il robot ha ruotato
        
            self.robot_pose[2]=dir
        else:  #il robot si muove 
            if(dir=="North"):
                self.robot_pose[0]-=dist
            elif(dir=="West"):
                self.robot_pose[1]-=dist
            elif(dir=="East"):
                self.robot_pose[1]+= dist
            elif(dir=="South"):
                self.robot_pose[0]+=dist
                
    def lidar_permutation(self,dir):
        #recupero valori sensori distanza e permutazione dell'ordine nel formato[Nord,Sud,Est,Ovest] in base alla direzione
        self.lidarsensor()
        sd=self.lidar_value
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
    
    def layer_reattivo(self,delta):#individuato oggetto a distanza delta
        
           
           self.lidarsensor()
           
           if(self.lidar_value[0]<delta): 
               return False  
           else:
               return True 
    
        
    def stop_motors(self):
        
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        
    def move(self,dist):#movimento in avanti
        
        maxspeed=self.MAX_SPEED
        self.odo()#aggiornamento valori dist_value
        start_dist=self.dist_values.copy()#distanza percorsa al momento della partenza del movimento
        
        while self.robot.step(self.timestep) != -1:
            
            self.odo()
            layer_reattivo=self.layer_reattivo(0.3)
            if self.dist_values[0]-start_dist[0]<=dist  and layer_reattivo:
                
                self.leftMotor.setVelocity(maxspeed)
                self.rightMotor.setVelocity(maxspeed)
                
            else:
                self.stop_motors()
                break
        
        self.robot_update(round(self.dist_values[0]-start_dist[0]))#aggiornamento posizione
        

    def move_back(self,dist):#movimento all'indietro
    
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
        
        #calcolo gradi da percorrere
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
            if(degree==999):#il robot punta già nella direzione richiesta
                break
            if(degree!=new_degree):
                self.leftMotor.setVelocity(left_speed)
                self.rightMotor.setVelocity(right_speed)
            else:
                self.stop_motors()
                break
             
        self.robot_update(0) #aggiornamernto direzione
        
    
    def follow_path(self,path):#movimento lungo il percorso di cella in cella
        
        for x,y in path[1:]:
          if((x-self.robot_pose[0])==0 and y>self.robot_pose[1] ):
              if(self.robot_pose[2]!="East"):
                  self.rotate("East")
                
              self.move(1)
          elif((x-self.robot_pose[0])==0 and y<self.robot_pose[1] ):
              if(self.robot_pose[2]!="West"):
                  self.rotate("West")
                 
              self.move(1)
          elif((y-self.robot_pose[1])==0 and x<self.robot_pose[0] ):
              if(self.robot_pose[2]!="North"):
                  self.rotate("North")
                  
              self.move(1)
          elif((y-self.robot_pose[1])==0 and x>self.robot_pose[0] ):
              if(self.robot_pose[2]!="South"):
                  self.rotate("South")
                 
              self.move(1)
          
        return True     
  
                 
    def find_path_obj(self,map,x,y):#calcolo percorso più breve
       
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
        
        return path[:-1]#percorso fino alla cella prima dell'ggetto
        
    def find_path_min(self,free_cells,x,y,map):
        
        min=100
        length=0
        paths=[]
        for cell in free_cells:#calcolo più percorsi e poi ne calcolo il minore
            grid = Grid(matrix=~abs(map).astype(bool))
            start = grid.node(y,x)
            end = grid.node(cell[1],cell[0])
            
            # Usa l'algoritmo A* per trovare il percorso
            finder = AStarFinder()
            path, runs = finder.find_path(start, end, grid)
            path = [(nodo.y, nodo.x) for nodo in path]
            paths.append(path)
        
        #calcolo percorso minimo
        for path in paths:
            length=len(path)
            if length<min:
                min=length
                min_path=path
        return min_path
        
    
    def position_recalculation(self):
        
        pos_est=[]#posizione stimata
        rb_list=[self.robot_pose[:2]]#lista delle poioni visitate
        count=2#penultima cella di rb_list
        while(self.PF.isLocalizated(0.5)):#il robot esce dal while quando avrà visitato 5 celle diverse
            
            
            sd=self.lidar_permutation(self.direction())
            #il robot si muove verso la prima cella adiacente libera non già visitata in senso orario
            if(sd[0]>1 and ([self.robot_pose[0]+(-1),self.robot_pose[1]] not in rb_list)):
               
                count=2
                self.rotate("North")
                self.move(1)
                pos_est=self.PF.particle_filter([-1,0],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
                
            elif(sd[2]>1 and ([self.robot_pose[0]+0,self.robot_pose[1]+1] not in rb_list)):
               
                count=2
                self.rotate("East")
                self.move(1)
                pos_est=self.PF.particle_filter([0,1],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
                
            elif(sd[3]>1 and ([self.robot_pose[0],self.robot_pose[1]+(-1)] not in rb_list)):
                
                count=2
                self.rotate("West")
                self.move(1)
                pos_est=self.PF.particle_filter([0,-1],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
                
            elif(sd[1]>1  and ([self.robot_pose[0]+1,self.robot_pose[1]] not in rb_list)):
                
                count=2
                self.rotate("South")
                self.move(1)
                pos_est=self.PF.particle_filter([1,0],self.lidar_permutation(self.direction()),self.direction())
                rb_list.append(self.robot_pose[:2])
            
            else:
                #se le celle adiacenti sono già state visitate ci muoviamo verso la cella precedente
                self.obj_dir(rb_list[-count][0],rb_list[-count][1])#direzionamento verso la l'ultima cella visitata
                azione=[rb_list[-count][0]-self.robot_pose[0],rb_list[-count][1]-self.robot_pose[1]]#calcolo azione di movimento
                self.move(1)
                pos_est=self.PF.particle_filter(azione,self.lidar_permutation(self.direction()),self.direction())
                count+=1
                
           
           
        return pos_est    
    
    def agg_filtro(self):
          print("Rilevata posizione non corretta, effettuo una localizzazione")
          
          # ridistribuzione particelle
          self.PF.redistribution()
          
          # movimento di almeno 6 celle al fine di ottenere la posizione corretta
          pos_est=self.position_recalculation() 
          self.robot_pose[:2]=pos_est
          self.robot_pose[2]=self.direction()
                  
                        
    def follow_path_filtered(self,path,map):
    
        layer_reattivo=True
        self.PF.map=map
      
        for x,y in path[1:]:
        
          sd_p=self.lidar_permutation(self.direction())
          
          if(self.robot_pose[:2] not in self.PF.position_estimate(sd_p,self.direction())):
                  
                  self.agg_filtro()
                  return False  
      
          if((x-self.robot_pose[0])==0 and y>self.robot_pose[1] ):
              if(self.robot_pose[2]!="East"):
                  self.rotate("East")
              if(self.layer_reattivo(1)==False):
 
                  self.agg_filtro()
                  return False  
                     
              self.move(1)
             
          elif((x-self.robot_pose[0])==0 and y<self.robot_pose[1] ):
              if(self.robot_pose[2]!="West"):
                  self.rotate("West")
              if(self.layer_reattivo(1)==False):
                  self.agg_filtro()
                  return False  
              self.move(1)
             
             
          elif((y-self.robot_pose[1])==0 and x<self.robot_pose[0] ):
              if(self.robot_pose[2]!="North"):
                  self.rotate("North")
              if(self.layer_reattivo(1)==False):
                  self.agg_filtro()
                  return False     
              self.move(1)
           
              
          elif((y-self.robot_pose[1])==0 and x>self.robot_pose[0] ):
              if(self.robot_pose[2]!="South"):
                  self.rotate("South")
              if(self.layer_reattivo(1)==False):
                  self.agg_filtro()
                  return False   
              self.move(1)
             
              
        return True