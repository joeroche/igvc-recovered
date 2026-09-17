#takes the vaues from controllerInputs and maps them 
#to the values that we want to write to the motor controller

#has multiple control schemes:
#two stick: one stick per motor
#tank: one stick for forward/backward, one stick for turning (WIP)
#racing: triggers for throttle, left stick for steering (WIP)

from scipy.interpolate import interp1d

maxSpeed = 40  # find whatever the max speed of the motor is
deadzone = 0.1
mapHigh = interp1d([0,1], [0, maxSpeed])
mapLow = interp1d([-1,0], [-1*maxSpeed, 0])

def mapControlsTwoStick(values):
    output = {}

    if (abs(values['leftJoystickY']) > deadzone):
        if(values['leftJoystickY'] > 0):
            output['leftMotorTargetSpeed'] = round(float(mapHigh(values['leftJoystickY'])))
        
        if(values['leftJoystickY'] < 0):
            output['leftMotorTargetSpeed'] = round(float(mapLow(values['leftJoystickY'])))
    else:
            output['leftMotorTargetSpeed'] = 0

    if(abs(values['rightJoystickY']) > deadzone):
        if(values['rightJoystickY'] > 0):
            output['rightMotorTargetSpeed'] = round(float(mapHigh(values['rightJoystickY'])))
        
        if(values['rightJoystickY'] < 0):
            output['rightMotorTargetSpeed'] = round(float(mapLow(values['rightJoystickY'])))
    
    else:
        output['rightMotorTargetSpeed'] = 0

    return output

def mapControlsTank(values): #WIP
    return
    
def mapControlsRacing(values): #WIP
    return
