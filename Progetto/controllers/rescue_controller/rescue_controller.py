
from controller import Robot, DistanceSensor, Motor
import math
from Movement import Movement
from Mapping import Mapping
from Cam import Cam
from Collect import Collect
from TTS import TTS
from Particle_filter import Particle_filter

TIME_STEP = 32


# create the Robot instance.

robot = Robot()
# get the time step of the current world.
timestep=TIME_STEP
#timestep = int(robot.getBasicTimeStep())
pos_start=[5,1]#posizione iniziale 5.1
dim_map=[5,15]  #dimensioni mappa
n_particles=300 
                  
def main():
    tts=TTS(robot)
    cam=Cam(robot,timestep)
    movement=Movement(robot,timestep,pos_start[0],pos_start[1])
    mapping=Mapping(movement,cam,dim_map[0],dim_map[1],pos_start[0],pos_start[1],tts)
    collect=Collect(movement,cam,robot,pos_start,tts)
    P_F=Particle_filter(n_particles,movement)
    while robot.step(TIME_STEP) != -1:
         
          
          
          map=mapping.mapping()
          P_F.map=map
          
          path=movement.find_path_obj(map,3,2)
          P_F.follow_path_filtered(path,P_F.particles)
          #collect.map=map
          #collect.start_collect()
          
          break
     
          
          
          
           


if __name__ == "__main__":
    main()



