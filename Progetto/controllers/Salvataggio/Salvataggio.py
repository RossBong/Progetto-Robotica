
from controller import Supervisor

# create the Robot instance.
robot = Supervisor() 


timestep = 32

rescuer = robot.getFromDef('rescuer')


customDataField=rescuer.getField('customData')
while robot.step(timestep) != -1:
    customData=customDataField.getSFString()
    if(customData=="5,5" or customData=="5,6"):
        robot.getFromDef('umano1').remove()
        print("Umano 1 salvato")
    elif(customData=="3,13" or customData=="4,13"):
        robot.getFromDef('umano2').remove()
        print("Umano 2 salvato")
    customDataField.setSFString(" ")