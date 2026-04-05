import time
import odrive 
from fibre.protocol import ChannelBrokenException
from odrive.enums import MOTOR_ERROR_CONTROL_DEADLINE_MISSED,  AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_POSITION_CONTROL,INPUT_MODE_TRAP_TRAJ
odrv0 = odrive.find_any()
try:
    odrv0.reboot()
except ChannelBrokenException:
    pass  # expected: device reset, link gone

time.sleep(2)
odrv0 = odrive.find_any()
odrv0.axis0.clear_errors()
odrv0.axis0.requested_state = AXIS_STATE_IDLE  # IDLE
odrv0.axis0.clear_errors()
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL # Mode Position CONTROL_MODE_VELOCITY_CONTROL CONTROL_MODE_TORQUE_CONTROL
odrv0.axis0.controller.config.input_mode = 1    # Mode Direct
odrv0.axis0.requested_state = 4
time.sleep(5)
odrv0.axis0.requested_state = 7
time.sleep(10)

odrv0.axis0.controller.input_pos = odrv0.axis0.encoder.pos_estimate
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL  

odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ  # trap
# Limites de la rampe (à tuner)
odrv0.axis0.trap_traj.config.vel_limit = 1600.0    # tours/s max sur la traj
odrv0.axis0.trap_traj.config.accel_limit = 400.0  # (tours/s²) typique à ajuster
odrv0.axis0.trap_traj.config.decel_limit = 400.0

while True:
	odrv0.axis0.controller.input_pos = odrv0.axis0.controller.input_pos+60
	print(odrv0.axis0.controller.input_pos)

	time.sleep(24)
