
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
    
    
    
    
rescuer = robot.getFromDef('rescuer')


customDataField=rescuer.getField('customData')
customData=customDataField.getSFString()
customDataField.setSFString(" ")
aggiungi_umano([4,0,0.13],[0,1,0,-1.57],robot,'umano1')
aggiungi_umano([12,1.19,0.18],[-0.577,0.577,-0.577,-2.09],robot,'umano2')

while robot.step(timestep) != -1:
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



    