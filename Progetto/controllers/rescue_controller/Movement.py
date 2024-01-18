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
        
        """
        #distance sensor instance
        self.frontsx= robot.getDevice('so3')
        self.frontdx= robot.getDevice('so4')
        self.frontsx.enable(timestep)
        self.frontdx.enable(timestep)
        self.backdx= robot.getDevice('so11')
        self.backsx= robot.getDevice('so12')
        self.backsx.enable(timestep)
        self.backdx.enable(timestep)
        self.left= robot.getDevice('so0')
        self.left.enable(timestep)
        self.right= robot.getDevice('so7')
        self.right.enable(timestep)
        """
        
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
        
        self.sd_value=[0,0,0,0,0,0]#[frontdx,frontsx,bacdx,backsx,left,right]
        
        self.circonf_ruota=self.raggio_ruota*2*math.pi #0.61 m
        self.enc_unit=self.circonf_ruota/6.29 #porzione di circonferenza per radiante
        
        self.robot_pose=[x_start,y_start,"North"]
        
        self.lidar_value=[[],[],[],[]]
        
   
        
    def sensordistance(self):
       self.sd_value[0]=self.sonar_to_m(self.frontdx.getValue())
       self.sd_value[1]=self.sonar_to_m(self.frontsx.getValue())
       self.sd_value[2]=self.sonar_to_m(self.backdx.getValue())
       self.sd_value[3]=self.sonar_to_m(self.backsx.getValue()) 
       self.sd_value[4]=self.sonar_to_m(self.left.getValue())
       self.sd_value[5]=self.sonar_to_m(self.right.getValue()) 
       
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
            
      
               
           
           
    """          
    def sonar_to_m(self,val):
        # Punti dati
        x1, y1 = 0, 1024  # (distanza in metri, valore del sensore)
        x2, y2 = 5, 0     # (distanza in metri, valore del sensore)
    
        # Calcola il coefficiente angolare (m) e l'intercetta (b)
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
    
        # Calcola la distanza in metri utilizzando l'equazione della retta
        distanza_metri = (val - b) / m
    
        return distanza_metri"""
    
      
    def get_time(self,distance, speed):
        rad=(distance*2*3.14)/(self.circonf_ruota)
        return rad/speed
        
    def stop_motors(self):
        
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        print("Motors stopped.")
        
    def move2(self,dist):
        maxspeed=self.MAX_SPEED
        seconds = self.get_time(dist, maxspeed)
        end_time = seconds + self.robot.getTime()
        print(f"Moving {dist} m forward...")
        
        while self.robot.step(self.timestep) != -1:
            
            if self.robot.getTime() < end_time and self.layer_reattivo():
                
                self.leftMotor.setVelocity(maxspeed)
                self.rightMotor.setVelocity(maxspeed)
                
            else:
                self.stop_motors()
                break
           
        self.robot_update(dist)
        
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
        
    def raggiungi(self):
    
        self.move(3.0)
        self.rotate("East")
        self.move(6.0)
        
        self.rotate("South")
        
        self.move(3.0)
        
        self.rotate("West")
        
        
        self.move(2.0)
        
        self.rotate("North")
        
        self.move(1.0)
        
        self.rotate("West")
        
        self.move(1.0)
        
        self.rotate("South")
        self.move(1.0)
        self.rotate("West")
        self.move(3.0)
        self.rotate("North")
    
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
        
        
        
        
        
        
        
        
        
        
        
      
    #last_ps_values=[0,0]
    
    """
    def robot_position():
        ps_values[0]=left_ps.getValue()
        ps_values[1]=right_ps.getValue()
        
        print('-----------------------')
        print("position sensor value:"+str(ps_values[0])+" "+str(ps_values[1]))
        
        dist_values[0]=ps_values[0]*enc_unit
        dist_values[1]=ps_values[1]*enc_unit
        print("distance value:"+str(dist_values[0])+" "+str(dist_values[1]))
        q=(imu.getRollPitchYaw()[2] * 180) / 3.14159
        print(f"imu:{q}")"""