from controller import  Camera, CameraRecognitionObject

class Cam:

    def __init__(self,robot,timestep):
        self.camera = robot.getDevice('camera')
        self.camera.enable(timestep)
        self.camera.recognitionEnable(timestep)
        
    def recognition(self):
        objs = self.camera.getRecognitionObjects()
        
        if len(objs) > 0:
                # get the first object
                obj = objs[0]
                print(obj)