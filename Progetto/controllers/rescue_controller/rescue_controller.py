from controller import Robot, DistanceSensor, Motor
import math
from Movement import Movement
from Mapping import Mapping
from Cam import Cam
from Collect import Collect
from TTS import TTS
from Particle_filter import Particle_filter



robot = Robot() # istanza Rescuerbot

timestep=32

pos_start=[5,1] #posizione iniziale 5.1
dim_map=[5,15]  #dimensioni mappa

                  
def main():
    tts=TTS(robot) #istanza text to speach
    cam=Cam(robot,timestep) #istanza camera
    P_F=Particle_filter() #istanza filtro particellare
    movement=Movement(robot,timestep,pos_start[0],pos_start[1],P_F) #istanza classe movimento
    mapping=Mapping(movement,cam,dim_map[0],dim_map[1],pos_start[0],pos_start[1],tts) #istanza classe mapping
    collect=Collect(movement,cam,robot,pos_start,tts) #istanza raccolta
    
    while robot.step(timestep) != -1:
          
          map=mapping.mapping()#effettuiamo il mapping
          collect.map=map#passiamo la mappa all'oggetto collect
          collect.start_collect()#effettuiamo la raccolta degli oggetti
          
          break
          

if __name__ == "__main__":
    main()



