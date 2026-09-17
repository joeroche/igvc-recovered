#Handles reading inputs from a controller.
import pygame

class ControllerReader():
    def __init__(self):
        self.controller = None
        self.joystick = None
        self.controllerName = None
        self.numAxes = None
        self.numButtons = None
        self.rightJoystickX = 0
        self.rightJoystickY = 0
        self.leftJoystickX = 0
        self.leftJoystickY = 0
        self.leftTrigger = 0
        self.rightTrigger = 0

    def openController(self):
        pygame.init()
        pygame.joystick.init()
        
        self.joystick = pygame.joystick.Joystick(0)  
        self.joystick.init()

        self.controllerName = self.joystick.get_name()
        self.numAxes = self.joystick.get_numaxes()
        self.numButtons = self.joystick.get_numbuttons()

        print(f"Joystick name: {self.controllerName}")
        print(f"Number of axes: {self.numAxes}")
        print(f"Number of buttons: {self.numButtons}")

    def closeController(self):
        del self.controller

    def updateInputs(self):
        report = {}

        if self.joystick:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.leftJoystickX = self.joystick.get_axis(0)
                    self.leftJoystickY = self.joystick.get_axis(1)
                    self.rightJoystickX = self.joystick.get_axis(2) 
                    self.rightJoystickY = self.joystick.get_axis(3)
            
            report = {
                        "leftJoystickX": self.leftJoystickX,
                        "leftJoystickY": self.leftJoystickY,
                        "rightJoystickX": self.rightJoystickX,
                        "rightJoystickY": self.rightJoystickY
                    }

        return report