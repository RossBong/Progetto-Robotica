import math
from controller import Robot, DistanceSensor, Motor
class Movement:

    MAX_SPEED = 4
    
    ps_values=[0, 0] #radianti relativi all'odometria
    dist_values=[0,0] #distanza percorsa misurata con l'odometria
    
    raggio_ruota=0.19/2
    
    dist_ruote=0.34215
    d_mid=dist_ruote/2
    
    sd_value=[0,0,0,0]
    
    circonf_ruota=raggio_ruota*2*math.pi #0.61 m
    enc_unit=circonf_ruota/6.29 #porzione di circonferenza per radiante
    
    robot_pose=[0,0," "]
    
    
    def __init__(self,robot,timestep):
        self.robot=robot
        self.timestep=timestep
        leftMotor = robot.getDevice('left wheel')
        rightMotor = robot.getDevice('right wheel')
    
    
        leftMotor.setPosition(float('inf'))
        rightMotor.setPosition(float('inf'))
    
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
    
        #position sensor instance
        left_ps= robot.getDevice('left wheel sensor')
        left_ps.enable(timestep)
        right_ps= robot.getDevice('right wheel sensor')
        right_ps.enable(timestep)
        
        #distance sensor instance
        frontsx= robot.getDevice('so3')
        frontdx= robot.getDevice('so4')
        frontsx.enable(timestep)
        frontdx.enable(timestep)
        backdx= robot.getDevice('so11')
        backsx= robot.getDevice('so12')
        backsx.enable(timestep)
        backdx.enable(timestep)
        
        #enable imu
        imu = robot.getDevice('inertial unit')
        imu.enable(timestep)
        
   
        
        
    

    
    
       
    def direction():
        q=(imu.getRollPitchYaw()[2] * 180) / 3.14159
        if (q<= -135 and q >= -180) or (135 <= q <= 180):
            return "West"
        elif q <= -45 and q > -135:
            return "South"
        elif 45 <= q <= 135:
            return "North"
        elif (-45 < q <= 0) or (0 <= q < 45):
            return "East" 
        
    def robot_update(dist):
        
        
        global robot_pose
        dir=direction()
        if (dist==0):  #il robot ha ruotato
        
            robot_pose[2]=dir
        else:  #il robot si muove di una casella nella precedente direzione
            if(dir=="North"):
                robot_pose[0]+=dist
            elif(dir=="West"):
                robot_pose[1]-=dist
            elif(dir=="East"):
                robot_pose[1]+= dist
            elif(dir=="South"):
                robot_pose[0]-=dist
        print(f"posizione:({robot_pose[0]},{robot_pose[1]}), direzione:{robot_pose[2]}")
    
    def layer_reattivo():
        
           global sd_value
           sd_value[0]=sonar_to_m(frontdx.getValue())
           sd_value[1]=sonar_to_m(frontsx.getValue())
           sd_value[2]=sonar_to_m(backdx.getValue())
           sd_value[3]=sonar_to_m(backsx.getValue())
           
           delta=0.300
           
           
           if(sd_value[0]<delta or sd_value[1]<delta):
               print(f"Oggetto rilevato a distanza {sd_value[0]}m")
               return False
               
           else:
               return True
            
      
               
           
           
              
    def sonar_to_m(val):
        # Punti dati
        x1, y1 = 0, 1024  # (distanza in metri, valore del sensore)
        x2, y2 = 5, 0     # (distanza in metri, valore del sensore)
    
        # Calcola il coefficiente angolare (m) e l'intercetta (b)
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
    
        # Calcola la distanza in metri utilizzando l'equazione della retta
        distanza_metri = (val - b) / m
    
        return distanza_metri
    
      
    def get_time(distance, speed):
        rad=(distance*2*3.14)/(circonf_ruota)
        return rad/speed
        
    def stop_motors():
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        print("Motors stopped.")
        
    def move(dist,forward):
        maxspeed=MAX_SPEED
        seconds = get_time(dist, MAX_SPEED)
        end_time = seconds + robot.getTime()
        print(f"Moving {dist} m forward...")
        if(forward==False):
            maxspeed=-MAX_SPEED
        while robot.step(timestep) != -1:
            
            if robot.getTime() < end_time and layer_reattivo():
                
                leftMotor.setVelocity(maxspeed)
                rightMotor.setVelocity(maxspeed)
                
            else:
                stop_motors()
                break
        if(dist>0.257):        
            robot_update(dist)
        
      
                
                
    
        
    def rotate(dir):
        last_dir=direction()
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
       
        while robot.step(timestep) != -1:
            
            new_degree=round((imu.getRollPitchYaw()[2] * 180) / 3.14159)
            
            if(degree!=new_degree):
                
                leftMotor.setVelocity(left_speed)
                rightMotor.setVelocity(right_speed)
            else:
                stop_motors()
                break
             
        robot_update(0)
        
    def raggiungi():
    
        move(3.0,True)
        rotate("East")
        move(6.0,True)
        
        rotate("South")
        
        move(3.0,True)
        
        rotate("West")
        
        
        move(2.0,True)
        
        rotate("North")
        
        move(1.0,True)
        
        rotate("West")
        
        move(1.0,True)
        
        rotate("South")
        move(1.0,True)
        rotate("West")
        move(3.0,True)
        rotate("North")
        
        
        
        
        
        
        
        
        
        
        
        
      
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