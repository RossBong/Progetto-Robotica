import numpy as np
import matplotlib.pyplot as plt
from controller import Robot


class Particle_filter:

    def __init__(self):
    
        self.map=[]
        x, y = np.meshgrid(np.arange(0,5,0.25),np.arange(0,15,0.25))
        self.particles = np.column_stack((x.ravel(), y.ravel()))
        self.n_particles=len(self.particles)
        
    def redistribution(self):
        x, y = np.meshgrid(np.arange(0,5,0.25),np.arange(0,15,0.25))
        self.particles = np.column_stack((x.ravel(), y.ravel()))
        


    def print_particles(self,particles):
    
        
        num_rows = 5
        num_columns = 15
       
        # Creare una figura e assi
        fig, ax = plt.subplots()
        
        ax.imshow(self.map, cmap='Blues', origin='upper', extent=[0, num_columns+1, num_rows+1, 0], alpha=0.5)
        
        plt.scatter(particles[:, 1], particles[:,0], alpha=0.5)
        plt.title('Scatter plot delle particelle')
        plt.xlabel('Coordinata Y')
        plt.ylabel('Coordinata X')
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
    
    def lidar_permutation(self,sd,dir):
    
        #self.movement.lidarsensor()
        #sd=self.movement.lidar_value #North,South,East,West 
        
        #dir =self.robot_pose[2]
        
        if(dir=="North"):
            return sd
        elif(dir=="East"):
            sd_p=[sd[3],sd[2],sd[0],sd[1]]
            return sd_p
        elif(dir=="South"):
            sd_p=[sd[1],sd[0],sd[3],sd[2]]
            return sd_p
        elif(dir=="West"):
            sd_p=[sd[2],sd[3],sd[1],sd[0]]
            return sd_p
    
    
    
    def position_estimate(self,sd,dir):
         
       
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
    def particle_filter(self, azione,sd,dir):
    
        particelle=self.particles
        rumore_movimento = np.array([0.5, 0.5])**2
        rumore_misurazione = 0.05
        #0.1**2 
        
        # incremento x ed y
        particelle += azione
            
        pesi = 1/len(particelle)
        
        for pos in self.position_estimate(sd,dir):
            nb=(np.linalg.norm(pos - particelle, axis=1)) ** 2
            pesi += np.exp(-(nb) / rumore_misurazione)
            pesi /= np.sum(pesi)
            
        # Resampling con copia di elementi più probabili   
        indici = np.random.choice(range(self.n_particles), self.n_particles, p=pesi)
        
        particelle_probabili = particelle[indici]# particelle più probabili
        position_estimated=np.mean(particelle_probabili, axis=0)
        position_estimated[0]=round(position_estimated[0])
        position_estimated[1]=round(position_estimated[1])
        position_estimated=position_estimated.astype(int)
        position_estimated=[position_estimated[0],position_estimated[1]]
        self.particles=particelle_probabili
      
        print("posizione stimata "+str(position_estimated))
        #self.print_particles(particelle_probabili)
      
        
        return position_estimated
        
        
        """
    
                  
                  """
              
            
        
   