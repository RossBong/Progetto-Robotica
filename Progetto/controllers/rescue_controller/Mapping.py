import math
from Movement import Movement
from Cam import Cam
import numpy as np
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
    
    def mapping(self):
        self.visited[self.x_start][self.y_start]=True #posizione di partenza visitata
        self.map[self.x_start][self.y_start]=0 #posizione di partenza libera
        self.map[:, 0] = 1 #muro ovest
        self.map[:, -1] = 1 #muro est
        self.map[0, :] = 1 #muro nord
        self.map[-1, :] = 1 #muro sud
        
        
        while(-1 in self.map):
            
            
            
            x=self.movement.robot_pose[0]
            y=self.movement.robot_pose[1]
            
            self.visited[x,y]=True 
            if(self.movement.direction()!="North"):
                self.movement.rotate("North")
            self.movement.sensordistance()
            sd=self.movement.sd_value
            
            print("Cam")
            self.cam.recognition()
            print("___________")
            #North
            if(sd[0]>1 or sd[1]>1):
                self.map[x-1,y]=0
                
            elif(sd[0]<1 or sd[1]<1):
                self.map[x-1,y]=1
                
            
            #East
            if(sd[5]>1):
                self.map[x,y+1]=0
            elif(sd[5]<1):
                self.map[x,y+1]=1
                
            
            #South
            if(sd[2]>1 or sd[3]>1):
                self.map[x+1,y]=0
            elif(sd[2]<1 or sd[3]<1):
                self.map[x+1,y]=1
                
            
            #West
            if(sd[4]>1):
                self.map[x,y-1]=0
            elif(sd[4]<1):
                self.map[x,y-1]=1
                
                
            print(self.map)
            #print(self.visited)
            
            if(self.map[x-1,y]==0 and self.visited[x-1,y]==False):
                
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
                path=self.find_path_min(free_cells,x,y)
                print(path)
                self.movement.follow_path(path)
                
        path=self.find_path_min([(self.x_start,self.y_start)],x,y)
        print(path)
        self.movement.follow_path(path) 
        self.movement.rotate("North")      
            
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
            
            start = grid.node(y,x)
            end = grid.node(cell[1],cell[0])
            
            # Usa l'algoritmo A* per trovare il percorso
            finder = AStarFinder()
            path, runs = finder.find_path(start, end, grid)
            path = [(nodo.y, nodo.x) for nodo in path]
            paths.append(path)
            #grid.cleanup()
        
        #calcolo percorso minimo
        for path in paths:
            length=len(path)
            if length<min:
                min=length
                min_path=path
        return min_path
            
        
            
        
