import math
from controller import Robot, DistanceSensor, Motor, Lidar
class Movement:

    
    
    
    def __init__(self,robot,timestep,x_start,y_start):
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
        print(f"posizione:({self.robot_pose[0]},{self.robot_pose[1]}), direzione:{self.robot_pose[2]}")
    
    def layer_reattivo(self):
        
           
           self.lidarsensor()
           
           delta=0.300
           
           
           if(self.lidar_value[0]<delta):
               print(f"Oggetto rilevato a distanza {self.lidar_value[0]}m")
               return False
               
           else:
               return True
            
      
               
           
           
   
    
      
    
        
    def stop_motors(self):
        
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        print("Motors stopped.")
        

        
    def move(self,dist):
        maxspeed=self.MAX_SPEED
        self.odo()
        start_dist=self.dist_values.copy()
        
        
        print(f"Moving {dist} m forward...")
        
        while self.robot.step(self.timestep) != -1:
            self.odo()
            if self.dist_values[0]-start_dist[0]<=dist  and self.layer_reattivo():
                
                self.leftMotor.setVelocity(maxspeed)
                self.rightMotor.setVelocity(maxspeed)
                
            else:
                self.stop_motors()
                break
        print(f"odo: {self.dist_values[0]-start_dist[0]}")  
        self.robot_update(dist)
      
                
                
    
        
    def rotate(self,dir):
        last_dir=self.direction()
        left_speed=0.5
        right_speed=-0.5
        if(dir=="North"):
            if(last_dir=="East"):
                left_speed=-0.5
                right_speed=0.5
            degree=90
        elif(dir=="South"):
            if(last_dir=="West"):
                    left_speed=-0.5
                    right_speed=0.5
        
            degree=-90
        elif(dir=="East"):
            if(last_dir=="South"):
                    left_speed=-0.5
                    right_speed=0.5
            degree=0
        elif(dir=="West"):
            if(last_dir=="North"):
                    left_speed=-0.5
                    right_speed=0.5
            degree=180
        print(f"Rotating {dir}...")
       
        while self.robot.step(self.timestep) != -1:
            
            new_degree=round((self.imu.getRollPitchYaw()[2] * 180) / 3.14159)
            
            if(degree!=new_degree):
                
                self.leftMotor.setVelocity(left_speed)
                self.rightMotor.setVelocity(right_speed)
            else:
                self.stop_motors()
                break
             
        self.robot_update(0)#aggiornamernto direzione
        
    
    def follow_path(self,path):
        
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
        
        
  