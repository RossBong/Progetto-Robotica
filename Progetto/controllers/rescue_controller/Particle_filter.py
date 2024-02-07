import numpy as np
import matplotlib.pyplot as plt
from controller import Robot


class Particle_filter:

    def __init__(self,length,width):
    
        self.map=[]
        self.length=length
        self.width=width
        # creazione griglia particellare uniforme  rispetto dimensioni mappa 
        x, y = np.meshgrid(np.arange(0,width+1,0.25),np.arange(0,length+1,0.25))
        self.particles = np.column_stack((x.ravel(), y.ravel()))
        self.n_particles=len(self.particles)# numero particelle
        
        
    def redistribution(self):
        # ridistribuzione uniforme delle particelle
        x, y = np.meshgrid(np.arange(0,self.width+1,0.25),np.arange(0,self.length+1,0.25))
        self.particles = np.column_stack((x.ravel(), y.ravel()))
        self.print_particles(self.particles)# stampa grafico particellare rispetto la mappa

    def print_particles(self,particles):
        # presentazione grafica delle particelle distribuite nella mappa       
        fig, ax = plt.subplots()
        ax.imshow(self.map, cmap='Blues', origin='upper', extent=[0, self.length+1, self.width+1, 0], alpha=0.5)
        plt.scatter(particles[:, 1], particles[:,0], alpha=0.5)
        plt.title('Scatter plot delle particelle')
        plt.xlabel('Coordinata Y')
        plt.ylabel('Coordinata X')
        plt.show()

      
    def real_state(self,cell): #verifica se le celle poste ai 4 punti cardinali rispetto 
                               #una data posizione sono libere
        
        state=[]#[North,South,East,Weast]
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
    #date le misurazioni dei 4 sensori di distanza 
    #restituisce 1 se la cella è occupata oppure 0 se è libera     
        mis_list=[]
        for mis in misurations:
            if mis>1:
                mis_list.append(0)
            else:
                mis_list.append(1)
                
        return mis_list   
    
    
    def position_estimate(self,sd,dir):#calcola le celle che rappresentano la posizione più probabile
         
        sd=self.evaluate_mis(sd)# stato cella attuale rispetto i sensori lidar
        positions_estimated=[]# posizione stimata
        
        # valutiamo gli stati compatibili di ogni cella della mappa
        # rispetto lo stato della posizione attuale
        for i in range(self.map.shape[0]-1):
            for j in range(self.map.shape[1]-1):
              
                cell_state=self.real_state([i,j])#stato cella rispetto mappa
                if(cell_state==sd):
                    # celle più probabili
                    positions_estimated.append([i,j])
                 
        return positions_estimated


    def particle_filter(self, azione,sd,dir):
        # Funzione per l'aggiornamento del filtro a particelle
        
        particelle=self.particles
        
        # movimento del robot
        particelle += azione
        
        # inizializzazione pesi uguali per ogni particella( particelle equiprobabili)  
        pesi = np.ones(len(particelle))*1/len(particelle)
        
        #calcolo pesi rispetto posizioni probabili
        #associando pesi maggiori a particelle vicine ad esse
        eps=0.0000001
        for pos in self.position_estimate(sd,dir):
           
            dist=(np.linalg.norm(pos - particelle, axis=1))+eps
            # maschera per selezionare particelle a distanza minore di 1m
            #dalla posizione probabile
            nb=dist[dist <= 1]
                                
            pesi[dist <= 1] +=1/nb #aggiornamento pesi
            
        pesi /= np.sum(pesi)#normalizzazione pesi
            
        # Campionamento per importanza  prendendo maggiormente elementi più probabili   
        indici = np.random.choice(range(self.n_particles), self.n_particles, p=pesi)

        particelle_probabili = particelle[indici]# particelle più probabili
        
        # posizione stimata rispetto particelle più probabili
        position_estimated=np.mean(particelle_probabili, axis=0)
        
        position_estimated[0]=round(position_estimated[0])
        position_estimated[1]=round(position_estimated[1])
        position_estimated=position_estimated.astype(int)
        position_estimated=[position_estimated[0],position_estimated[1]]
        
        self.particles=particelle_probabili# aggiornamento particelle
      
        print("Posizione stimata: "+str(position_estimated))
        
        self.print_particles(particelle_probabili)

        return position_estimated
        
        
    def isLocalizated(self, raggio):
        # metodo per valutare se il robot è riuscito a localizzarsi
        
        # centroide delle particelle
        centroide = np.mean(self.particles, axis=0)
        
        # distanza euclidea tra particelle e centroide
        distanze = np.linalg.norm(self.particles - centroide, axis=1)
        
        # percentuale di particelle entro il raggio definito
        percentuale = np.sum(distanze < raggio) / distanze.size
        
        if percentuale<=0.99:
            return True #localizzazione non ancora precisa
        else:
            return False #localizzazione effettuata
            

        