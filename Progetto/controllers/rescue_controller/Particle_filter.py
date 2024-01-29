import numpy as np
import matplotlib.pyplot as plt
from controller import Robot
from Movement import Movement 

class Particle_filter:

    def __init__(self,n_particles,movement):
    
        self.map=[]
        self.movement=movement
        self.robot_pose=self.movement.robot_pose
        self.particles = np.random.normal(7, 3, (n_particles, 2))
        self.n_particles=n_particles


    def print_particles(self):
        
        plt.figure(figsize=(10, 6))
        plt.scatter(self.particles[:, 0], self.particles[:, 1], alpha=0.5)
        plt.title('Scatter plot delle particelle')
        plt.xlabel('Coordinata X')
        plt.ylabel('Coordinata Y')
        plt.show()
      
    def real_state(self,cell):
        
        state=[]
        x_robot,y_robot=cell[0],cell[1]
        if (self.map[x_robot-1,y_robot]==4 or self.map[x_robot-1,y_robot]==3 ):#North
            state.append(1)
        else:
            state.append(0)
            
        if (self.map[x_robot+1,y_robot]==4 or self.map[x_robot+1,y_robot]==3 ):#South
            state.append(1)
        else:
            state.append(0)
            
        if (self.map[x_robot,y_robot+1]==4 or self.map[x_robot,y_robot+1]==3 ):#East
            state.append(1)
        else:
            state.append(0)
               
        if (self.map[x_robot,y_robot-1]==4 or self.map[x_robot,y_robot-1]==3 ):#Weast
            state.append(1)
        else:
            state.append(0)
         
        return state
    
    def evaluate_mis(self,misurations):
        mis_list=[]
        for mis in misurations:
            if mis>1:
                mis_list.append(0)
            else:
                mis_list.append(1)
                
        return mis_list   
    
    def position_estimate(self):
        self.movement.lidarsensor()
        sd=self.movement.lidar_value      
   
        sd=self.evaluate_mis(sd)
        positions_estimated=[]
        for i in range(self.map.shape[0]-1):
            for j in range(self.map.shape[1]-1):
                cell = self.map[i, j]
                cell_state=self.real_state([i,j])
                if(cell_state==sd):
                    # celle più probabili
                    positions_estimated.append([i,j])
                    
        return positions_estimated
                    
                    
    
    
    # Funzione per l'aggiornamento del filtro a particelle
    def start_particle_filter(self, particelle,azione):    
        rumore_movimento = np.array([0.5, 0.5])**2
        rumore_misurazione = 0.1**2 
        
        # incremento x ed y
        particelle += azione
            
        pesi = 1/len(particelle)
        
        for pos in self.position_estimate():
            print("posizione dentro for  "+str(pos)) 
            # Peso delle particelle basato sulla misurazione
            sub=np.array(self.robot_pose[:2])-np.array( pos)
             
            distanza_misurata = np.linalg.norm(sub)    
            pesi += np.exp(-((distanza_misurata - np.linalg.norm(pos - particelle, axis=1)) ** 2) / rumore_misurazione)
            pesi /= np.sum(pesi)
            
        # Resampling con copia di elementi più probabili   
        indici = np.random.choice(range(self.n_particles), self.n_particles, p=pesi)
        
        particelle_probabili = particelle[indici]# particelle più probabili
        position_estimated=np.mean(particelle_probabili, axis=0)
        self.particles=particelle_probabili
        self.print_particles()
        return position_estimated,particelle_probabili
        
    def follow_path_filtered(self,path,parts):
            layer_reattivo=True
            particles = parts
            pos_est=[]
            for x,y in path[1:]:
              if((x-self.robot_pose[0])==0 and y>self.robot_pose[1] ):
                  if(self.robot_pose[2]!="East"):
                      self.movement.rotate("East")
                   
                  self.movement.move(1)
                  pos_est,particles=self.start_particle_filter(particles,[0,1])
                  
              elif((x-self.robot_pose[0])==0 and y<self.robot_pose[1] ):
                  if(self.robot_pose[2]!="West"):
                      self.movement.rotate("West")
                 
                  self.movement.move(1)
                  pos_est,particles=self.start_particle_filter(particles,[0,-1])
              elif((y-self.robot_pose[1])==0 and x<self.robot_pose[0] ):
                  if(self.robot_pose[2]!="North"):
                      self.movement.rotate("North")
                    
                  self.movement.move(1)
                  pos_est,particles=self.start_particle_filter(particles,[-1,0])
              elif((y-self.robot_pose[1])==0 and x>self.robot_pose[0] ):
                  if(self.robot_pose[2]!="South"):
                      self.movement.rotate("South")
                 
                  self.movement.move(1)
                  pos_est,particles=self.start_particle_filter(particles,[1,0])
                  
              print("posizione stimata "+str(pos_est))
            return True
        
   