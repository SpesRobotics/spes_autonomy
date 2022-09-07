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


def main(axis='axis1'):
    do_anticogging = ('--anticogging' in sys.argv)

    odrv0 = odrive.find_any()

    print('Voltage:', odrv0.vbus_voltage)
    odrv0 = save_configuration(odrv0, erase=True)

    getattr(odrv0, axis).motor.config.pole_pairs = 15

    getattr(odrv0, axis).motor.config.resistance_calib_max_voltage = 4
    getattr(odrv0, axis).motor.config.requested_current_range = 25
    getattr(odrv0, axis).motor.config.current_control_bandwidth = 100
    getattr(odrv0, axis).motor.config.current_lim = 4.0
    getattr(odrv0, axis).motor.config.calibration_current = 4.0

    getattr(odrv0, axis).encoder.config.mode = ENCODER_MODE_HALL
    getattr(odrv0, axis).encoder.config.cpr = 90
    getattr(odrv0, axis).encoder.config.bandwidth = 100
    getattr(odrv0, axis).encoder.config.use_index = True

    getattr(odrv0, axis).controller.config.pos_gain = 1.0
    getattr(odrv0, axis).controller.config.vel_gain = 0.11
    getattr(odrv0, axis).controller.config.vel_integrator_gain = 1.6
    getattr(odrv0, axis).controller.config.vel_limit = 10
    getattr(odrv0, axis).controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0 = save_configuration(odrv0)

    print('Calibrating motor...')
    getattr(odrv0, axis).requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while getattr(odrv0, axis).requested_state != AXIS_STATE_UNDEFINED or getattr(odrv0, axis).current_state != AXIS_STATE_IDLE:
        time.sleep(1)
    print('Motor calibration complete', getattr(odrv0, axis).error)
    getattr(odrv0, axis).motor.config.pre_calibrated = True
    getattr(odrv0, axis).encoder.config.pre_calibrated = True
    odrv0 = save_configuration(odrv0)

    if do_anticogging:
        print('Calibrating anti-cogging...')
        getattr(odrv0, axis).encoder.config.use_index = True
        getattr(odrv0, axis).controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        getattr(odrv0, axis).controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        getattr(odrv0, axis).requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        getattr(odrv0, axis).controller.config.anticogging.calib_pos_threshold = 1.0
        getattr(odrv0, axis).controller.config.anticogging.calib_vel_threshold = 1.0
        getattr(odrv0, axis).controller.start_anticogging_calibration()
        while getattr(odrv0, axis).controller.config.anticogging.calib_anticogging:
            time.sleep(1)
        print('Anti-cogging calibration complete')
        getattr(odrv0, axis).controller.config.anticogging.pre_calibrated = True
        odrv0 = save_configuration(odrv0)

    print('Setting a sample speed...')
    getattr(odrv0, axis).requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    getattr(odrv0, axis).requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    getattr(odrv0, axis).controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    getattr(odrv0, axis).controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    getattr(odrv0, axis).controller.input_vel = 0.1
    print('Axis in closed loop control')
    time.sleep(10)
    getattr(odrv0, axis).requested_state = AXIS_STATE_IDLE


if __name__ == '__main__':
    main('axis0')
    main('axis1')
