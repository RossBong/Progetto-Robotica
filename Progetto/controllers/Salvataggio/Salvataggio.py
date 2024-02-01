
from controller import Supervisor

# create the Robot instance.
robot = Supervisor() 


timestep = 32

def aggiungi_umano(traslation,rotation,robot,name):
    root=robot.getRoot()
    chil=root.getField('children')
    chil.importMFNodeFromString(-1,'DEF '+name+' Pedestrian_new {}')
    
    umano=robot.getFromDef(name)
    traslationField=umano.getField('translation')
    traslationField.setSFVec3f(traslation)
    
    rotationField=umano.getField('rotation')
    rotationField.setSFRotation(rotation)

def aggiungi_box(traslation,color_rec,robot,name,nameproto):
    root=robot.getRoot()
    chil=root.getField('children')
    chil.importMFNodeFromString(-1,'DEF '+name+nameproto)
    
    box=robot.getFromDef(name)
    traslationField=box.getField('translation')
    traslationField.setSFVec3f(traslation)

def recupera_oggetti(objs) :
    
    for obj in objs:
        
        if(obj.getPosition()[0]<1 and obj.getPosition()[1]<1 and obj.getPosition()[2]<=0.075):
            

            obj.remove()
            objs.remove(obj)
             
def salvataggio_umano(rescuer):    
    customDataField=rescuer.getField('customData')
    customData=customDataField.getSFString()
    
    
    if(customData=="5,5" or customData=="5,6"):
        robot.getFromDef('umano1').remove()
        print("Umano 1 salvato")
        customDataField.setSFString(" ")
    elif(customData=="3,13" or customData=="4,13"):
        robot.getFromDef('umano2').remove()
        print("Umano 2 salvato")
        customDataField.setSFString(" ")
    
    
rescuer = robot.getFromDef('rescuer')
customDataField=rescuer.getField('customData')
customData=customDataField.getSFString()
customDataField.setSFString(" ")

aggiungi_umano([4,0,0.13],[0,1,0,-1.57],robot,'umano1')
aggiungi_umano([12,1.19,0.18],[-0.577,0.577,-0.577,-2.09],robot,'umano2')

aggiungi_box([1,2,0.1],[1,1,0],robot,'box_gioielli',' Box_gioielli {}')
aggiungi_box([1,4,0.1],[0,1,0.5],robot,'box_soldi1',' Box_soldi {}')
aggiungi_box([11,0,0.1],[0,1,0],robot,'box_foto',' Box_foto {}')
aggiungi_box([14,2,0.1],[0,1,0.5],robot,'box_soldi2',' Box_soldi {}')

objs=[robot.getFromDef('box_gioielli'),robot.getFromDef('box_soldi1'),robot.getFromDef('box_soldi2'),robot.getFromDef('box_foto')]
   
while robot.step(timestep) != -1:

    salvataggio_umano(rescuer)    
    recupera_oggetti(objs)


    