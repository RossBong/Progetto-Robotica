"""rescue_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import math

TIME_STEP = 32
MAX_SPEED = 4

# create the Robot instance.
robot = Robot()

# get the time step of the current world.

timestep = int(robot.getBasicTimeStep())

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


ps_values=[0, 0] #radianti relativi all'odometria
dist_values=[0,0] #distanza percorsa misurata con l'odometria

raggio_ruota=0.19/2

dist_ruote=0.34215
d_mid=dist_ruote/2

dist_value=[0,0,0,0]

circonf_ruota=raggio_ruota*2*math.pi #0.61 m
enc_unit=circonf_ruota/6.29 #porzione di circonferenza per radiante

robot_pose=[0,0," "]
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

def raddrizza():
    
       global dist_value
       dist_value[0]=sonar_to_m(frontdx.getValue())
       dist_value[1]=sonar_to_m(frontsx.getValue())
       dist_value[2]=sonar_to_m(backdx.getValue())
       dist_value[3]=sonar_to_m(backsx.getValue())
       max=0.500
       min=0.252
       
       if(dist_value[0]<min):
           move(min-dist_value[0],False)
           print(f"Ci siamo mossi di {min-dist_value[0]} indietro")
       elif(dist_value[1]<min):
           move(min-dist_value[1],False)
           print(f"Ci siamo mossi di {min-dist_value[1]} indietro")
       elif(dist_value[2]<min):
           move(min-dist_value[2],True)
           print(f"Ci siamo mossi di {min-dist_value[2]} in avanti")    
  
       elif(dist_value[3]<min):
           move(min-dist_value[3],True)
           print(f"Ci siamo mossi di {min-dist_value[3]} in avanti")
           
       if(dist_value[0]>max and dist_value[0]<1):
           move(dist_value[0]-max,False)
           print(f"Ci siamo mossi di {dist_value[0]-max} indietro")
       elif(dist_value[1]>max and dist_value[1]<1):
           move(dist_value[1]-max,False)
           print(f"Ci siamo mossi di {dist_value[1]-max} indietro")
       elif(dist_value[2]>max and dist_value[2]<1):
           move(dist_value[2]-max,True)
           print(f"Ci siamo mossi di {dist_value[2]-max} in avanti")    
  
       elif(dist_value[3]>max and dist_value[3]<1):
           move(dist_value[3]-max,True)
           print(f"Ci siamo mossi di {dist_value[3]-max} in avanti")
           
       
       
          
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


"""  
def distancesensor():
  
       global dist_value
       dist_value[0]=sonar_to_m(frontdx.getValue())
       dist_value[1]=sonar_to_m(frontsx.getValue())
       dist_value[2]=sonar_to_m(backdx.getValue())
       dist_value[3]=sonar_to_m(backsx.getValue())
     
       print(dist_value)
"""    
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
    while robot.step(TIME_STEP) != -1:
        
        if robot.getTime() < end_time:
            leftMotor.setVelocity(maxspeed)
            rightMotor.setVelocity(maxspeed)
            
        else:
            stop_motors()
            break
    if(dist>0.257):        
        robot_update(dist)
    
  
            
            
def get_rot_speed_rad(degrees, seconds, raggio_ruota, d_mid):
    circle = d_mid * 2 * math.pi
    dist = (degrees / 360) * circle
    linear_vel = dist / seconds
    left_wheel_speed = linear_vel / raggio_ruota
    right_wheel_speed = -1 * linear_vel /raggio_ruota
    return left_wheel_speed, right_wheel_speed
    
               
def rotate(degrees, seconds, direction):

    
    # get the left and right rotaional speeds to turn x degrees in y seconds
    left, right = get_rot_speed_rad(degrees, seconds, raggio_ruota, d_mid)
    end_time = seconds + robot.getTime()
    print(f"Rotating {direction}...")
    
    while robot.step(TIME_STEP) != -1:
        print((imu.getRollPitchYaw()[2] * 180) / 3.14159)
        if robot.getTime() < end_time:
            leftMotor.setVelocity(left)
            rightMotor.setVelocity(right)
        else:
            stop_motors()
            break
    raddrizza()      
    robot_update(0)
    
            
def raggiungi():
    move(3.0,True)
    
    rotate(90, 1.5, "right")
    
    
    move(6.0,True)
    
    rotate(90, 1.5, "right")
    
    move(3.0,True)
    
    rotate(90, 1.5, "right")
    
    
    move(2.0,True)
    
    rotate(90, 1.5, "right")
    
    move(1.0,True)
    
    rotate(-90, 1.5, "right")
    
    move(1.0,True)
    
    rotate(-90, 1.5, "right")
    move(1.0,True)
    rotate(90, 1.5, "right")
    move(3.0,True)
    rotate(90, 1.5, "right")
    
                  
def main():

    while robot.step(TIME_STEP) != -1:
      
          raggiungi()
          #rotate(90, 1.5, "right")
          
          
           


if __name__ == "__main__":
    main()



