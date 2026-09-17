import odrive
from odrive.enums import *
from odrive.utils import *
import controlParse
import controllerInputs
import time

def init():
    print("finding odrive...")

    try:
        odrv0 = odrive.find_any()
        print("found odrive!")
        odrv0.clear_errors()
    except KeyboardInterrupt:
            print("Exiting...")
            
    encoder0PreCalibrated = odrv0.axis0.encoder.config.pre_calibrated
    encoder1PreCalibrated = odrv0.axis1.encoder.config.pre_calibrated
    motor0PreCalibrated = odrv0.axis0.motor.config.pre_calibrated
    motor1PreCalibrated = odrv0.axis1.motor.config.pre_calibrated
    PreCalibrated  = [encoder0PreCalibrated, encoder1PreCalibrated, motor0PreCalibrated, motor1PreCalibrated]

    if not all(PreCalibrated):
        odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("calibrating axis0...")

        while odrv0.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("calibrating axis1...")
        while odrv0.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        
        print("calibration complete!")

    print("setting defaults:")
    odrv0.config.dc_max_negative_current = -30
    odrv0.axis0.motor.config.current_lim = 60
    odrv0.axis0.motor.config.pole_pairs = 3
    odrv0.axis0.motor.config.torque_constant = 0.05
    odrv0.axis0.encoder.config.cpr = 4096
    odrv0.axis0.controller.config.input_filter_bandwidth = 2.0
    odrv0.axis0.controller.config.vel_integrator_gain = 0.16 # or something
    odrv0.axis0.controller.config.vel_gain = 0.045
    odrv0.axis0.controller.config.vel_limit = 1000
    odrv0.axis0.controller.config.vel_ramp_rate = 30
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    
    
    odrv0.axis1.motor.config.current_lim = 60
    odrv0.axis1.motor.config.pole_pairs = 3
    odrv0.axis1.motor.config.torque_constant = 0.05
    odrv0.axis1.encoder.config.cpr = 4096
    odrv0.axis1.controller.config.input_filter_bandwidth = 2.0
    odrv0.axis1.controller.config.vel_integrator_gain = 0.16 # or something
    odrv0.axis1.controller.config.vel_gain = 0.045
    odrv0.axis1.controller.config.vel_limit = 1000
    odrv0.axis1.controller.config.vel_ramp_rate = 30
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis1.controller.config.input_mode = InputMode.VEL_RAMP

    return odrv0

if __name__ == "__main__":
    myReader = controllerInputs.ControllerReader()
    myReader.openController()
    odrv0 = init()
    print("starting!")

    while(True):
        try:
            output = controlParse.mapControlsTwoStick(myReader.updateInputs())
            odrv0.axis1.controller.input_vel = -1*output['rightMotorTargetSpeed']
            odrv0.axis0.controller.input_vel = output['leftMotorTargetSpeed']
            #oprint(odrv0.axis0.motor.current_control.Iq_measured)
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except:
            print("ODrive disconnected! Reconnecting...")
            odrv0 = init()
        
        
