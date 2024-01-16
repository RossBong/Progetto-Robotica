
from controller import Robot, DistanceSensor, Motor
import math
from Movement import Movement

TIME_STEP = 32


# create the Robot instance.

robot = Robot()
# get the time step of the current world.
timestep=TIME_STEP
#timestep = int(robot.getBasicTimeStep())
    
                  
def main():
    movement=Movement(robot,timestep)
   
    while robot.step(TIME_STEP) != -1:
      
          movement.raggiungi()
          #rotate2("West")
          
          
          
          
           


if __name__ == "__main__":
    main()



