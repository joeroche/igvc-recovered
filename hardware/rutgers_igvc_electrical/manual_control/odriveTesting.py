import controllerInputs
import controlParse

if __name__ == "__main__":
    myReader = controllerInputs.ControllerReader()
    myReader.openController()
    rightMotorTargetSpeed = 0
    leftMotorTargetSpeed = 0
    print("starting!")

    while True:
        output = controlParse.mapControlsTwoStick(myReader.updateInputs())
        rightMotorTargetSpeed = output['rightMotorTargetSpeed']
        leftMotorTargetSpeed = -1 * output['leftMotorTargetSpeed']
        
        if(abs(rightMotorTargetSpeed) > 0):
            print("right motor speed: ")
            print(rightMotorTargetSpeed)
            
        if(abs(leftMotorTargetSpeed) > 0):
            print("left motor speed: ")
            print(leftMotorTargetSpeed)
