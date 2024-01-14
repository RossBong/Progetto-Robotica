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
ds1=robot.getDevice('ds1')
ds1.enable(timestep)
#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)


ps_values=[0, 0] #radianti relativi all'odometria
dist_values=[0,0] #distanza percorsa misurata con l'odometria

raggio_ruota=0.19/2

dist_ruote=0.33205
d_mid=dist_ruote/2

dist_value=[0,0,0,0,0]

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
"""
def raddrizza():
    
       global dist_value
       dist_value[0]=frontdx.getValue()/1000
       dist_value[1]=frontsx.getValue()/1000
       dist_value[2]=backdx.getValue()/1000
       dist_value[3]=backsx.getValue()/1000
      
       print(dist_value)
       if(dist_value[0]<0.257):
           move(0.257-dist_value[0],False)
           print(f"Ci siamo mossi di {0.257-dist_value[0]} in dietro")
       elif(dist_value[1]<0.257):
           move(0.257-dist_value[1],False)
           print(f"Ci siamo mossi di {0.257-dist_value[1]} in dietro")
       elif(dist_value[2]<0.257):
           move(0.257-dist_value[2],True)
           print(f"Ci siamo mossi di {0.257-dist_value[2]} in avanti")    
  
       elif(dist_value[3]<0.257):
           move(0.257-dist_value[3],True)
           print(f"Ci siamo mossi di {0.257-dist_value[3]} in avanti")
       
 """          
     
def distancesensor():
  
       global dist_value
       dist_value[0]=frontdx.getValue()
       dist_value[1]=frontsx.getValue()
       dist_value[2]=backdx.getValue()
       dist_value[3]=backsx.getValue()
       dist_value[4]=backsx.getValue()
       print(dist_value)
    
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
        distancesensor()
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
        
        if robot.getTime() < end_time:
            leftMotor.setVelocity(left)
            rightMotor.setVelocity(right)
        else:
            stop_motors()
            break
            
    robot_update(0)
    
            
def raggiungi():
    move(3.0,True)
   
    rotate(90, 1.5, "right")
    move(1.0,True)
 
    rotate(-90, 1.5, "right")
    
    move(1.0,True)
    
    rotate(180, 3, "right")
    move(2.0,True)
    rotate(90, 1.5, "right")
    
    move(1.0,True)
    rotate(-90, 1.5, "right")
    move(2.0,True)
    rotate(180, 3, "right")
    
                  
def main():

    while robot.step(TIME_STEP) != -1:
      
          raggiungi()
          #rotate(90, 1.5, "right")
          
           


if __name__ == "__main__":
    main()



