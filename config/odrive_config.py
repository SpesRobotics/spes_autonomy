# Install:
#
#     pip3 install odrive==0.5.4 fibre
#     echo "export PATH=\${PATH}:\${HOME}/.local/bin" >> ~/.bashrc
#     sudo -E $HOME/.local/bin/odrivetool udev-setup
#
# Reference: https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/docs/hoverboard.md

import time
import sys
import odrive
from odrive.enums import *


def save_configuration(odrv0, erase=False):
    odrv_instance = odrv0
    try:
        if erase:
            odrv_instance.erase_configuration()
        else:
            odrv_instance.save_configuration()
    except Exception:
        odrv_instance = None
    while odrv_instance is None:
        time.sleep(0.3)
        odrv_instance = odrive.find_any()
    try:
        odrv_instance.reboot()
    except Exception:
        odrv_instance = None
    while odrv_instance is None:
        time.sleep(0.3)
        odrv_instance = odrive.find_any()
    return odrv_instance


def main():
    do_anticogging = ('--anticogging' in sys.argv)

    odrv0 = odrive.find_any()

    print('Voltage:', odrv0.vbus_voltage)
    odrv0 = save_configuration(odrv0, erase=True)

    odrv0.axis1.motor.config.pole_pairs = 15

    odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
    odrv0.axis1.motor.config.requested_current_range = 25
    odrv0.axis1.motor.config.current_control_bandwidth = 100
    odrv0.axis1.motor.config.current_lim = 4.0
    odrv0.axis1.motor.config.calibration_current = 4.0

    odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
    odrv0.axis1.encoder.config.cpr = 90
    odrv0.axis1.encoder.config.bandwidth = 100
    odrv0.axis1.encoder.config.use_index = True

    odrv0.axis1.controller.config.pos_gain = 1.0
    odrv0.axis1.controller.config.vel_gain = 0.02
    odrv0.axis1.controller.config.vel_integrator_gain = 0.1
    odrv0.axis1.controller.config.vel_limit = 10
    odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0 = save_configuration(odrv0)

    print('Calibrating motor...')
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while odrv0.axis1.requested_state != AXIS_STATE_UNDEFINED or odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(1)
    print('Motor calibration complete', odrv0.axis1.error)
    odrv0.axis1.motor.config.pre_calibrated = True
    odrv0.axis1.encoder.config.pre_calibrated = True
    odrv0 = save_configuration(odrv0)

    if do_anticogging:
        print('Calibrating anti-cogging...')
        odrv0.axis1.encoder.config.use_index = True
        odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        odrv0.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv0.axis1.controller.config.anticogging.calib_pos_threshold = 1.0
        odrv0.axis1.controller.config.anticogging.calib_vel_threshold = 1.0
        odrv0.axis1.controller.start_anticogging_calibration()
        while odrv0.axis1.controller.config.anticogging.calib_anticogging:
            time.sleep(1)
        print('Anti-cogging calibration complete')
        odrv0.axis1.controller.config.anticogging.pre_calibrated = True
        odrv0 = save_configuration(odrv0)

    print('Setting a sample speed...')
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis1.controller.input_vel = 0.1
    print('Axis in closed loop control')
    time.sleep(10)
    odrv0.axis1.requested_state = AXIS_STATE_IDLE


if __name__ == '__main__':
    main()
