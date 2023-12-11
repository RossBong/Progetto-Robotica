"""rescue_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import math

TIME_STEP = 64
MAX_SPEED = 6.29

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = TIME_STEP

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

ps_values=[0, 0]
dist_values=[0,0]

raggio_ruota=0.195/2
dist_ruote=0.192

circonf_ruota=raggio_ruota*2*3.14  #0.61 m
enc_unit=circonf_ruota/6.29

robot_pose=[0,0,0]
last_ps_values=[0,0]

def robot_position():
    ps_values[0]=left_ps.getValue()
    ps_values[1]=right_ps.getValue()
    
    print('-----------------------')
    print("position sensor value:"+str(ps_values[0])+" "+str(ps_values[1]))
    
    """for i in range(2):
        diff=ps_values[i]-last_ps_values[i]
        if diff<0.001:
            diff=0
            ps_values[i]=last_ps_values[i]
        dist_values[i]=diff*enc_unit
    #print("distance value:"+str(dist_values[0])+" "+str(dist_values[1]))
 
    v=(dist_values[0]+dist_values[1])/2.0
    w=(dist_values[0]-dist_values[1])/dist_ruote
    
    dt=1
    robot_pose[2]+=(w*dt)
    
    vx=v*math.cos(robot_pose[2])
    vy=v*math.sin(robot_pose[2])
    
    robot_pose[0] +=(vx*dt)
    robot_pose[1] +=(vy*dt)
    print("posizione robot:"+str(robot_pose))"""
    
    dist_values[0]=ps_values[0]*enc_unit
    dist_values[1]=ps_values[1]*enc_unit
    print("distance value:"+str(dist_values[0])+" "+str(dist_values[1]))
    
    
    
def get_time(distance, speed):
    rad=distance/circonf_ruota
    return rad/speed
    
def stop_motors():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    print("Motors stopped.")
    
def move(dist):
    seconds = get_time(dist, MAX_SPEED)
    end_time = seconds + robot.getTime()
    
    while robot.step(TIME_STEP) != -1:
       
        print(robot.getTime())
        robot_position()
        print(f"Moving {dist} m forward...")
        
        if robot.getTime() < end_time:
            leftMotor.setVelocity(MAX_SPEED)
            rightMotor.setVelocity(MAX_SPEED)
            
        else:
            stop_motors()
            break
            
def main():
    move(1.0)

    
if __name__ == "__main__":
    main()



