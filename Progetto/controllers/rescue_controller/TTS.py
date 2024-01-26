from controller import Robot,Speaker

class TTS:

    def __init__(self,robot,engine,language):
        self.speaker=robot.getDevice('speaker')
        self.speaker.setEngine(engine)
        self.speaker.setLanguage(language)

    def text_to_speech(self,testo):
        self.speaker.speak(testo,1.0)
        
        
    

