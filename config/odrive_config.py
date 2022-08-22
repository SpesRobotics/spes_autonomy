import time
import odrive
from odrive.enums import *


def save_configuration(odrv0):
    odrv0.save_configuration()
    try:
        odrv0.reboot()
    except Exception as e:
        pass
    time.sleep(2)
    return odrive.find_any()


def main():
    odrv0 = odrive.find_any()

    print('Voltage:', odrv0.vbus_voltage)

    # odrv0.config.enable_brake_resistor = True

    odrv0.axis1.motor.config.pole_pairs = 15

    odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
    odrv0.axis1.motor.config.requested_current_range = 25
    odrv0.axis1.motor.config.current_control_bandwidth = 100
    odrv0.axis1.motor.config.torque_constant = 8.27 / 16
    odrv0.axis1.motor.config.current_lim = 3

    odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
    odrv0.axis1.encoder.config.cpr = 90
    odrv0.axis1.encoder.config.calib_scan_distance = 150
    odrv0.axis1.encoder.config.ignore_illegal_hall_state = True

    odrv0.axis1.encoder.config.bandwidth = 100
    odrv0.axis1.controller.config.pos_gain = 1
    odrv0.axis1.controller.config.vel_gain = 0.02 * odrv0.axis1.motor.config.torque_constant * odrv0.axis1.encoder.config.cpr
    odrv0.axis1.controller.config.vel_integrator_gain = 0.1 * odrv0.axis1.motor.config.torque_constant * odrv0.axis1.encoder.config.cpr
    odrv0.axis1.controller.config.vel_limit = 10
    odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis1.motor.config.calibration_current = 3
    odrv0 = save_configuration(odrv0)

    print('Calibrating motor...')
    odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(10)
    print('Motor calibration complete', odrv0.axis1.motor.error)

    print('Calibrating encoder...')
    odrv0.axis1.motor.config.pre_calibrated = True
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(30)
    print('Encoder offset calibration complete', odrv0.axis1.encoder.error)
    odrv0.axis1.encoder.config.pre_calibrated = True
    odrv0 = save_configuration(odrv0)

    print('Setting a sample speed...')
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.input_vel = 0.1
    print('Axis in closed loop control', odrv0.axis1)
    time.sleep(10)
    odrv0.axis1.controller.input_vel = 0


if __name__ == '__main__':
    main()
