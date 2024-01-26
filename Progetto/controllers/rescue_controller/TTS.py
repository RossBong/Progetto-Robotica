from controller import Robot,Speaker

class TTS:

    def __init__(self,robot):
        self.speaker=robot.getDevice('speaker')
        self.speaker.setEngine('microsoft')
        self.speaker.setLanguage('it-IT')

    def text_to_speech(self,testo):
        self.speaker.speak(f"<prosody volume=100>{testo}</prosody>",1.0)