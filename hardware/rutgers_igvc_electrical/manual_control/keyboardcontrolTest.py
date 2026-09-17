from pynput import keyboard
from time import sleep
import odrive
from odrive.enums import *
import sys

def init():
    odrv0 = odrive.find_any()
    print(str(odrv0.vbus_voltage))

    if (odrv0.axis0.motor.config.pre_calibrated != True or odrv0.axis0.encoder.config.pre_calibrated != True):
    # calibrate
        odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(2)
        odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("not calibrated")


    print("setting defaults:")
    odrv0.config.dc_max_negative_current = -30

    odrv0.axis0.motor.config.current_lim = 30
    odrv0.axis0.motor.config.pole_pairs = 3
    odrv0.axis0.motor.config.torque_constant = 0.05
    odrv0.axis0.encoder.config.cpr = 4096
    odrv0.axis0.controller.config.input_filter_bandwidth = 2.0
    odrv0.axis0.controller.config.vel_integrator_gain = 0.16 # or something
    odrv0.axis0.controller.config.vel_gain = 0.045
    odrv0.axis0.controller.config.vel_limit = 60
    odrv0.axis0.controller.config.vel_ramp_rate = 75
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP

    odrv0.axis1.motor.config.current_lim = 30
    odrv0.axis1.motor.config.pole_pairs = 3
    odrv0.axis1.motor.config.torque_constant = 0.05
    odrv0.axis1.encoder.config.cpr = 4096
    odrv0.axis1.controller.config.input_filter_bandwidth = 2.0
    odrv0.axis1.controller.config.vel_integrator_gain = 0.16 # or something
    odrv0.axis1.controller.config.vel_gain = 0.045
    odrv0.axis1.controller.config.vel_limit = 60
    odrv0.axis1.controller.config.vel_ramp_rate = 75
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis1.controller.config.input_mode = InputMode.VEL_RAMP

    print('done!')
    return odrv0
def key_press(key):
    odrv0 = odrive.find_any()
    if key.char == 'w':
        odrv0.axis0.controller.input_vel = 30
        odrv0.axis1.controller.input_vel = -1 * 30

    if key.char == 's':
        odrv0.axis0.controller.input_vel = -30
        odrv0.axis1.controller.input_vel = -1 * -30

    if key.char == 'a':
        odrv0.axis0.controller.input_vel = 15
        odrv0.axis1.controller.input_vel = 15

    if key.char == 'd':
        odrv0.axis0.controller.input_vel = -15
        odrv0.axis1.controller.input_vel = -15

init()
listener = keyboard.Listener(on_press=key_press)
listener.start()


while True:
    sleep(0.01)