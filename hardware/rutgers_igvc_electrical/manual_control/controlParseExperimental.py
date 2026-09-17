from scipy.interpolate import interp1d
import controllerInputs
import time


maxSpeed = 60 # find whatever the max speed of the motor is
deadzone = 60 #maybe add ability for 2 seperate deadzones

mapHigh = interp1d([(255/2)+deadzone,255], [0, maxSpeed])
mapLow = interp1d([0,(255/2)-deadzone], [-1*maxSpeed, 0])

def mapControlsTwoStick(values): #one joystick for each wheel, holding a makes both wheels go forward
    output = {}
    if (values['a'] == True):
        output['rightMotorTargetSpeed'] = maxSpeed
        output['leftMotorTargetSpeed'] = maxSpeed
        return output
    else:
        if(values['firstY'] > (255/2)+deadzone):
            output['leftMotorTargetSpeed'] = round(-1*mapHigh(values['firstY']))
        elif(values['firstY'] < (255/2)-deadzone):
            output['leftMotorTargetSpeed'] = round(-1*mapLow(values['firstY'])) # this could probably be done just by multiplying by -1 but im tired rn
        else:
            output['leftMotorTargetSpeed'] = 0

        if(values['secondY'] > (255/2)+deadzone):
            output['rightMotorTargetSpeed'] = round(-1*mapHigh(values['secondY']))
        elif(values['secondY'] < (255/2)-deadzone):
            output['rightMotorTargetSpeed'] = round(-1*mapLow(values['secondY']))
        else:
            output['rightMotorTargetSpeed'] = 0
    #print(output)
    return output

def mapControlsTank(values): # i am copying this from https://xiaoxiae.github.io/Robotics-Simplified-Website/drivetrain-control/arcade-drive/
    output = {}
    rotate, drive = values['firstX'], 255-values['firstY']    #these 3 lines convert the map of values the controller gives out to the axes shown on the website
    rotate, drive = rotate-128, drive-128
    rotate, drive = rotate*2, drive*2
    print(drive)
    maximum = max(abs(rotate), abs(drive))
    total = drive + rotate
    difference = drive - rotate

    if drive >= 128 + deadzone:
        if rotate >= 128 + deadzone:
            output['leftMotorTargetSpeed'] = maximum
            output['rightMotorTargetSpeed'] = difference
        else:
            output['leftMotorTargetSpeed'] = total
            output['rightMotorTargetSpeed'] = maximum
    elif drive <= 128 - deadzone:
        if rotate >= 128 + deadzone:
            output['leftMotorTargetSpeed'] = total
            output['rightMotorTargetSpeed'] = -1*maximum
        else:
            output['leftMotorTargetSpeed'] = -1*maximum
            output['rightMotorTargetSpeed'] = difference
    else:
        output['leftMotorTargetSpeed'] = 0
        output['rightMotorTargetSpeed'] = 0

    output['leftMotorTargetSpeed'] = (output['leftMotorTargetSpeed']/255)*maxSpeed
    output['rightMotorTargetSpeed'] = (output['rightMotorTargetSpeed']/255)*maxSpeed

    return output

def mapControlsTrigger(values): # drive like forza, triggers are throttles, left joystick for steering
    output = {}

    return output
    # x axis of controller corresponds to a speed "x", leftSpeed=x, rightSpeed=-x. y axis works exactly the same as twostick. can it only be allowed to move forward and turn one at a time?
    # i didnt think about it but it would probably be able to turn just fine

# this program should be made into a class that can receive values from controllerInputs.py and get the actual values that we
# want to write to the motor controller. for now this just means mapping the y axes from the controller input to values between
# -maxSpeed to +maxSpeed

#actually in order to actually make use of the deadzones ill need 2 maps to start mapping where the deadzone ends on + and - side

# also i want 2 control schemes, one thats just one stick per motor and another thats ps1 tank controls

if(__name__ == "__main__"):
#    vals={'firstY': 127}
#    mapControllsTwoStick(vals)

    myDeviceName = "Controller (XBOX 360 For Windows)"
    # myDeviceName = "Mayflash WiiU Pro Game Controller Adapter"
    myReader = controllerInputs.ControllerReader()
    myReader.openController(myDeviceName)

    while(True):
        mapControllsTwoStick(myReader.updateInputs())
        time.sleep(0.05)


# acceleration should be proportional to the difference between target velocity and actual velocity