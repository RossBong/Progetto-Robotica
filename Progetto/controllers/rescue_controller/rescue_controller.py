
from controller import Robot, DistanceSensor, Motor
import math
from Movement import Movement
from Mapping import Mapping
from Cam import Cam
from Collect import Collect
from TTS import TTS
TIME_STEP = 32


# create the Robot instance.

robot = Robot()
# get the time step of the current world.
timestep=TIME_STEP
#timestep = int(robot.getBasicTimeStep())
pos_start=[5,1]#posizione iniziale 5.1
dim_map=[5,15]  #dimensioni mappa
                  
def main():
    tts=TTS(robot)
    cam=Cam(robot,timestep)
    movement=Movement(robot,timestep,pos_start[0],pos_start[1])
    mapping=Mapping(movement,cam,dim_map[0],dim_map[1],pos_start[0],pos_start[1],tts)
    collect=Collect(movement,cam,robot,pos_start)
    while robot.step(TIME_STEP) != -1:
         
          
          
          map=mapping.mapping()
          collect.map=map
          collect.start_collect()
          break
     
          
          
          
           


if __name__ == "__main__":
    main()



