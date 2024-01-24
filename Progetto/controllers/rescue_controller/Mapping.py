import math
from Movement import Movement
from Cam import Cam
import numpy as np
import random
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class Mapping:

    def __init__(self,movement,cam,width,length,x_start,y_start):
        
        self.movement=movement
        self.cam=cam
        self.map= -1 * np.ones((width+2, length+2))
        self.visited= False * np.ones((width+2, length+2))#True quando la cella è libera ed è stata visitata
        self.x_start=x_start
        self.y_start=y_start
        self.stato="Triste"
        self.counter_state=0#numero di celle senza aver trovato ogetti
        self.far_human=[0," "]#distanza e direzione

    
    def mapping(self):
        self.visited[self.x_start][self.y_start]=True #posizione di partenza visitata
        self.map[self.x_start][self.y_start]=0 #posizione di partenza libera
        self.map[:, 0] = 4 #muro ovest
        self.map[:, -1] = 4 #muro est
        self.map[0, :] = 4 #muro nord
        self.map[-1, :] = 4 #muro sud
        self.map= np.array([[4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4],
                   [4, 3, 0, 4, 4, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0, 4],
                   [4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4],
                   [4, 4, 3, 4, 4, 0, 0, 0, 0, 0, 0, 4, 4, 0, 4, 3, 4],
                   [4, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 0, 0, 4, 4],
                   [4, 0, 0, 0, 0, 0, 0, 0, 4, 4, 3, 0, 0, 4, 0, 4, 4],
                   [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]])
        self.visited=np.full((7, 17), True)
        
       
        
        while(-1 in self.map):
            
            
            
            x=self.movement.robot_pose[0]
            y=self.movement.robot_pose[1]
            
            self.visited[x,y]=True 
            if(self.movement.direction()!="North"):
                self.movement.rotate("North")
            #self.movement.sensordistance()
            self.movement.lidarsensor()
            sd=self.movement.lidar_value
            
            
            
           
            #North
            if(sd[0]>1):
                self.map[x-1,y]=0
                
            elif(sd[0]<1 and self.map[x-1,y]!=4 and self.map[x-1,y]!=3):
                    self.map[x-1,y]=1
                
            
            #East
            if(sd[2]>1):
                self.map[x,y+1]=0
            elif(sd[2]<1 and self.map[x,y+1]!=4 and self.map[x,y+1]!=3):
                self.map[x,y+1]=1
                
            
            #South
            if(sd[1]>1):
                self.map[x+1,y]=0
            elif(sd[1]<1 and self.map[x+1,y]!=4 and self.map[x+1,y]!=3):
                self.map[x+1,y]=1
                
            
            #West
            if(sd[3]>1):
                self.map[x,y-1]=0
            elif(sd[3]<1 and self.map[x,y-1]!=4 and self.map[x,y-1]!=3):
                self.map[x,y-1]=1
                
                 
            
            #print(self.visited)
            self.ricerca_ogg(x,y)
            print(self.map)
            if(self.far_human[0]>0):
                self.movement.rotate(self.far_human[1])
                self.movement.move(self.far_human[0])
                self.far_human=[0," "]
            elif(self.map[x-1,y]==0 and self.visited[x-1,y]==False):
                self.movement.rotate("North")
                self.movement.move(1)
                
            elif(self.map[x,y+1]==0 and self.visited[x,y+1]==False):
                self.movement.rotate("East")
                self.movement.move(1)
                
            elif(self.map[x+1,y]==0 and self.visited[x+1,y]==False):
                self.movement.rotate("South")
                self.movement.move(1)
                
            elif(self.map[x,y-1]==0 and self.visited[x,y-1]==False):
                self.movement.rotate("West")
                self.movement.move(1)
                
            else:
                free_cells=self.find_free_novisited()
                if(len(free_cells)==0):
                   print("Imposto i punti non raggiungibili a 4")
                   self.map=np.where(self.map == -1, 4, self.map) 
                else:
                    path=self.find_path_min(free_cells,x,y)
                    self.movement.follow_path(path)
                    
        lost_cells=np.argwhere(self.map == 1)          
        if(len(lost_cells)>0):
            
            print(f"Ops! Quando ero felice mi sono fatto prendere dall'euforia e non ho scansionato {len(lost_cells)} celle")
            self.stato="Normale"
            for cell in lost_cells:
            
                path=self.movement.find_path_obj(self.map,cell[0],cell[1])
               
                self.movement.follow_path(path)
                self.movement.obj_dir(cell[0],cell[1])
                self.scansione(cell[0],cell[1])
                
           
        
        print("Ho trovato tutti gli oggetti e tutti gli umani sono salvi!!") 
        print(self.map) 
        return self.map
        

          
    def scansione(self,x,y):
        trovato=False
        oggetto,dist=self.cam.recognition()
         
        if(self.stato=="Triste"):#permettiamo al robot di trovare ogetti più lontani
             if(oggetto=="umano"and dist>2 and self.map[x,y]==0 ):
                 print(f"Ho trovato un umano alla distanza di {dist}m")
                 trovato=True
                 self.far_human=[round(dist),self.movement.robot_pose[2]]
                 return trovato
                 
        if(oggetto=="umano" and dist<=2):
             print(f"Ho trovato un umano! Venite a recuperarlo.")
             trovato=True
             self.update_stato(trovato)
             self.rimuovi_umano(x,y)
             self.map[x,y]=0
        elif((oggetto=="box_gioielli"or oggetto=="box_soldi" or oggetto=="box_foto")and dist<1):
             self.map[x,y]=3
             print(f"Ho trovato il seguente oggetto:{oggetto}")
             trovato=True
             self.update_stato(trovato)
        elif(self.map[x,y]==1):
             self.map[x,y]=4
               
             
        return trovato
             
             
    def update_stato(self,f):
       
       if(f==True and self.stato=="Triste"):
           self.stato="Normale"
           self.counter_state=0
       elif(f==True and self.stato=="Normale"):
           self.stato="Felice"
           self.counter_state=0
       elif(f==False and self.stato=="Felice"and self.counter_state<3):
           self.counter_state+=1
       elif(f==False and self.stato=="Felice"and self.counter_state>=3):  
           self.stato='Normale'
           self.counter_state=0
       elif(f==False and self.stato=="Normale"and self.counter_state<3):
           self.counter_state+=1
       elif(f==False and self.stato=="Normale"and self.counter_state>=3):  
           self.stato='Triste'
           self.counter_state=0
       elif(f==True and self.stato=="Felice"):
           self.counter_state=0
       
           
             
                     
    def ricerca_ogg(self,x,y):
        s1,s2,s3,s4 =False,False,False,False
        if self.stato=="Normale":
            print("Mi sento bene, effettuo la scansione degli ogetti che ho rilevato in questa casella")
            if(self.map[x-1,y]==1):#North 
                 self.movement.rotate("North")
                 s1=self.scansione(x-1,y)  
                 
            if(self.map[x,y+1]==1):#East
                 self.movement.rotate("East")
                 s2=self.scansione(x,y+1)
                      
            if(self.map[x+1,y]==1):#South
                 self.movement.rotate("South")
                 s3=self.scansione(x+1,y)
                                 
            if(self.map[x,y-1]==1):#West
                 self.movement.rotate("West")
                 s4=self.scansione(x,y-1)
            
        elif self.stato=="Triste":
                 print("Sono Triste, non ho trovato nessun oggetto, cercherò in tutte le direzioni")
                 s1=self.scansione(x-1,y)
                 self.movement.rotate("East")
                 s2=self.scansione(x,y+1)
                 self.movement.rotate("South")
                 s3=self.scansione(x+1,y)
                 self.movement.rotate("West")
                 s4=self.scansione(x,y-1)
                 
        elif self.stato=="Felice":
                print("Sono Felice, ho trovato molti oggetti quindi andrò più velocemente")
                if(self.map[x-1,y]==1 and random.choice([0, 1])==1):#North    
                     s1=self.scansione(x-1,y)  
                     
                if(self.map[x,y+1]==1 and random.choice([0, 1])==1):#East
                     self.movement.rotate("East")
                     s2=self.scansione(x,y+1)
                          
                if(self.map[x+1,y]==1 and random.choice([0, 1])==1):#South
                     self.movement.rotate("South")
                     s3=self.scansione(x+1,y)
                                     
                if(self.map[x,y-1]==1 and random.choice([0, 1])==1):#West
                     self.movement.rotate("West")
                     s4=self.scansione(x,y-1)
                 
        if(s1==False and s2==False and s3==False and s4==False):
             self.update_stato(False)
         
                     
    def rimuovi_umano(self,x,y):
        self.movement.robot.setCustomData(f"{x},{y}")
        
        
       
                 
    def find_free_novisited(self):
        #funzione che restuisce una lista di coordinate delle celle
        #libere e non visitate
        a=self.map==self.visited
        free_cells=np.argwhere(a)
        return free_cells
    
    def find_path_min(self,free_cells,x,y):
        min=100
        length=0
        paths=[]
        for cell in free_cells:
            grid = Grid(matrix=~abs(self.map).astype(bool))
            print(np.array(~abs(self.map).astype(bool)))
            start = grid.node(y,x)
            end = grid.node(cell[1],cell[0])
            
            # Usa l'algoritmo A* per trovare il percorso
            finder = AStarFinder()
            path, runs = finder.find_path(start, end, grid)
            path = [(nodo.y, nodo.x) for nodo in path]
            paths.append(path)
            print(f"path:{path}")
            #grid.cleanup()
        
        #calcolo percorso minimo
        for path in paths:
            length=len(path)
            if length<min:
                min=length
                min_path=path
        return min_path
            
        
            
        
