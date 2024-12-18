import math
from Movement import Movement
from Cam import Cam
from TTS import TTS
import numpy as np
import random
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder#modulo A*


class Mapping:

    def __init__(self,movement,cam,width,length,x_start,y_start,tts):
        
        self.movement=movement #istanza oggetto della classe Movement
        self.cam=cam #istanza oggetto della classe Cam
        self.map= -1 * np.ones((width+2, length+2))#inizializziamo le posizioni a -1
        self.visited= False * np.ones((width+2, length+2)) #Inizializziamo le posizioni a False
        self.x_start=x_start#posizione di partenza
        self.y_start=y_start#posizione di partenza
        self.stato="Triste" #stato emotivo iniziale
        self.counter_state=0 #numero di celle senza aver trovato ogetti
        self.far_human=[0," "] #distanza e direzione umano individuato a distanza >1m
        self.tts=tts #istanza oggetto della classe TTS
    

           
    def rimuovi_umano(self,x,y):#impostiamo il campo del robot CustomData con le coordinate dell'umano
        self.movement.robot.setCustomData(f"{x},{y}")
                
    def find_free_novisited(self):
        #funzione che restuisce una lista di coordinate delle celle
        #libere e non visitate adiacenti
        free_visited_mask=self.map==self.visited
        free_cells=np.argwhere(free_visited_mask)
        return free_cells

        
    def print_info(self):
       
       print(f"posizione:({self.movement.robot_pose[0]},{self.movement.robot_pose[1]}), direzione:{self.movement.robot_pose[2]}")
       print("Mappa:")
       print(self.map)
       print(" ")
       print(" ")
       print(" ")
       print(" ")

    def scansione(self,x,y):
        trovato=False #oggetto o umano non trovato
        oggetto,dist=self.cam.recognition()#riconoscimento oggetto dalla camera
         
        if(self.stato=="Triste"):#permettiamo al robot di trovare umani più lontani
             if(oggetto=="umano"and dist>2 and self.map[x,y]==0 ):
                 txt=f"Ho trovato un umano alla distanza di {round(dist, 2)} metri, avvicinamento"
                 print(txt)
                 self.tts.text_to_speech(txt)
                 trovato=True
                 #aggiornamento posizione umano individuato
                 self.far_human=[round(dist),self.movement.robot_pose[2]]
                 return trovato
                 
        if(oggetto=="umano" and dist<=2):
             txt=f"Ho trovato un umano! Venite a recuperarlo."
             print(txt)
             self.tts.text_to_speech(txt)
             trovato=True
             self.update_stato(trovato)#aggiornamento stato emozionale
             self.rimuovi_umano(x,y)#comunicazione di rimozione al supervisor
             self.map[x,y]=0
        elif((oggetto=="box_gioielli"or oggetto=="box_soldi" or oggetto=="box_foto")and dist<1):
             self.map[x,y]=3
             txt=f"Ho trovato il seguente oggetto:{oggetto}"
             print(txt)
             self.tts.text_to_speech(txt)
             trovato=True
             self.update_stato(trovato)#aggiornamento stato emozionale
             
        elif(self.map[x,y]==1):#oggetto non riconosciuto es.roccia
             self.map[x,y]=4
             
        return trovato
             
             
    def update_stato(self,f):# aggiornamento stato emotivo
       
       if(f==True and self.stato=="Triste"):#oggetto trovato e stato Triste
           txt="Finalmente mi sento bene, effettuerò la scansione degli oggetti che rileverò in ogni casella"
           print(txt)
           self.tts.text_to_speech(txt)
           self.print_info()  
           self.stato="Normale"
           self.counter_state=0
       elif(f==True and self.stato=="Normale"):#oggetto trovato e stato Normale
           txt="Sono Felice, ho trovato molti oggetti quindi andrò più velocemente"
           print(txt)
           self.tts.text_to_speech(txt)
           self.print_info()  
           self.stato="Felice"
           self.counter_state=0
       elif(f==False and self.stato=="Felice"and self.counter_state<3):#oggetto non trovato, stato Felice e contatore<3
           print("Sono Felice")
           self.print_info()  
           self.counter_state+=1
       elif(f==False and self.stato=="Felice"and self.counter_state>=3): #oggetto non trovato, stato Felice e contatore=3
           txt="Non sono più Felice,starò più attento nella scansione degli oggetti"
           print(txt)
           self.tts.text_to_speech(txt)
           self.print_info()  
           self.stato='Normale'
           self.counter_state=0
       elif(f==False and self.stato=="Normale"and self.counter_state<3):#oggetto non trovato, stato Normale e contatore<3
           print("Sto Bene")
           self.print_info()  
           self.counter_state+=1
       elif(f==False and self.stato=="Normale"and self.counter_state>=3):#oggetto non trovato, stato Normale e contatore=3
           txt="Sono Triste, non ho trovato nessun oggetto, cercherò in tutte le direzioni"
           print(txt)
           self.tts.text_to_speech(txt)
           self.print_info()    
           self.stato='Triste'
           self.counter_state=0
       elif(f==True and self.stato=="Felice"):#oggetto trovato e stato Felice
           txt="Mi sento ancora Felice, quindi continuerò ad andare veloce"
           print(txt)
           self.tts.text_to_speech(txt)
           self.print_info()  
           self.counter_state=0
       elif(f==False and self.stato=="Triste"):#oggetto non trovato e stato Triste
           self.counter_state+=1
           txt=f"Sono ancora Triste, non tovo oggetti o umani da {self.counter_state} celle"
           print(txt)
           self.tts.text_to_speech(txt)
           self.print_info()  
             
                     
    def ricerca_ogg(self,x,y):#effettuo le scansioni in base allo stato
    
        s1,s2,s3,s4 =False,False,False,False#oggetti trovati per ogni scansione
        
        if self.stato=="Normale":#scansioni in direzione degli oggetti individuati
            
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
                 self.movement.rotate("Weast")
                 s4=self.scansione(x,y-1)
            
        elif self.stato=="Triste":#scansione in tutte le direzioni
                 
                 self.movement.rotate("North")
                 s1=self.scansione(x-1,y)
                 self.movement.rotate("East")
                 s2=self.scansione(x,y+1)
                 self.movement.rotate("South")
                 s3=self.scansione(x+1,y)
                 self.movement.rotate("Weast")
                 s4=self.scansione(x,y-1)
                 
        elif self.stato=="Felice":#scansioni in una porzione casuale di direzioni dove abbiamo individuato oggetti
                
                if(self.map[x-1,y]==1 and random.choice([0, 1])==1):#North 
                     self.movement.rotate("North")   
                     s1=self.scansione(x-1,y)  
                     
                if(self.map[x,y+1]==1 and random.choice([0, 1])==1):#East
                     self.movement.rotate("East")
                     s2=self.scansione(x,y+1)
                          
                if(self.map[x+1,y]==1 and random.choice([0, 1])==1):#South
                     self.movement.rotate("South")
                     s3=self.scansione(x+1,y)
                                     
                if(self.map[x,y-1]==1 and random.choice([0, 1])==1):#Weast
                     self.movement.rotate("Weast")
                     s4=self.scansione(x,y-1)
                 
        if(s1==False and s2==False and s3==False and s4==False):#se non troviamo oggetti in nessuna scansione
             self.update_stato(False)
             
             
             
    def mapping(self):
        self.visited[self.x_start][self.y_start]=True #posizione di partenza visitata
        self.map[self.x_start][self.y_start]=0 #posizione di partenza libera
        
       
        self.map= np.array([   [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4],
                               [4, 0, 3, 4, 4, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0, 4],
                               [4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4],
                               [4, 4, 3, 4, 4, 0, 0, 0, 0, 0, 0, 4, 4, 0, 4, 3, 4],
                               [4, 0, 4, 4, 0, 0, 4, 0, 4, 0, 0, 0, 4, 0, 0, 4, 4],
                               [4, 0, 0, 0, 0, 0, 0, 0, 4, 4, 0, 0, 3, 4, 0, 4, 4],
                               [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]])
        self.visited=np.full((7, 17), True)   

        while(-1 in self.map):#finchè nella mappa vi sono posizioni non identificate
            
            x=self.movement.robot_pose[0]
            y=self.movement.robot_pose[1]
            
            self.visited[x,y]=True #cella visitata

            #recupero valori sensori distanza e permutazione dell'ordine nel formato[Nord,Sud,Est,Ovest] in base alla direzione
            sd=self.movement.lidar_permutation(self.movement.direction())
            
            #North
            if(sd[0]>1):
                self.map[x-1,y]=0#cella libera
                
            elif(sd[0]<1 and self.map[x-1,y]!=4 and self.map[x-1,y]!=3):
                    self.map[x-1,y]=1  #cella occupata ma non identificata        
            
            #East
            if(sd[2]>1):
                self.map[x,y+1]=0#cella libera
            elif(sd[2]<1 and self.map[x,y+1]!=4 and self.map[x,y+1]!=3):
                self.map[x,y+1]=1#cella occupata ma non identificata  
            
            #South
            if(sd[1]>1):
                self.map[x+1,y]=0#cella libera
            elif(sd[1]<1 and self.map[x+1,y]!=4 and self.map[x+1,y]!=3):
                self.map[x+1,y]=1#cella occupata ma non identificata  
            
            #Weast
            if(sd[3]>1):
                self.map[x,y-1]=0#cella libera
            elif(sd[3]<1 and self.map[x,y-1]!=4 and self.map[x,y-1]!=3):
                self.map[x,y-1]=1#cella occupata ma non identificata  

            self.ricerca_ogg(x,y)#ricerca oggetti tramite camera
            
            if(self.far_human[0]>0):#rilevato umano a distanza >1m
                self.movement.rotate(self.far_human[1])#rotazione verso l'umano
                self.movement.move(self.far_human[0])#movimento verso l'umano
                self.far_human=[0," "]
                
            #movimenti verso cella libera non visitata adiacente
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
                self.movement.rotate("Weast")
                self.movement.move(1)
            
            #ricerca celle libere non visitate non adiacenti    
            else:
                free_cells=self.find_free_novisited()
                if(len(free_cells)==0): # celle libere visitate
                   print("Imposto i punti non raggiungibili a 4")
                   
                   self.map=np.where(self.map == -1, 4, self.map)#imposto le posizioni non raggiungibili a 4
                   self.print_info()  
                else:
                    # celle libere non ancora visitate
                    path=self.movement.find_path_min(free_cells,x,y,self.map)#calcolo percorso verso la più vicina cella libera
                    self.movement.follow_path(path)#movimento verso cella libera
                    
        lost_cells=np.argwhere(self.map == 1)#celle non analizzate a causa dello stato Felice          
        if(len(lost_cells)>0):
            
            txt=f"Ops! Quando ero felice mi sono fatto prendere dall'euforia e non ho scansionato {len(lost_cells)} celle"
            print(txt)
            self.tts.text_to_speech(txt)
            self.stato="Normale"
            for cell in lost_cells:# visita celle non analizzate
            
                path=self.movement.find_path_obj(self.map,cell[0],cell[1])
                self.movement.follow_path(path)
                self.movement.obj_dir(cell[0],cell[1])
                self.scansione(cell[0],cell[1])
                self.print_info() 
                
        txt="Ho trovato tutti gli oggetti e tutti gli umani sono salvi!!"
        print(txt)
        self.tts.text_to_speech(txt) 
        
        return self.map