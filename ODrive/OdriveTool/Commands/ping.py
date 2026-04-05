import time
import odrive
from odrive.enums import MOTOR_ERROR_CONTROL_DEADLINE_MISSED

DEADLINE = MOTOR_ERROR_CONTROL_DEADLINE_MISSED
POLL_S = 0.001  # 1 ms

TIMING_FIELDS = [
    "general", "adc_cb_i", "adc_cb_dc", "meas_r", "meas_l",
    "enc_calib", "idx_search", "foc_voltage", "foc_current",
    "spi_start", "sample_now", "spi_end",
]

def dump_timing_log(motor, label):
    tl = motor.timing_log
    print(f"--- {label} ---")
    for name in TIMING_FIELDS:
        print(f"  {name}: {getattr(tl, name)}")

def main():
    odrv0 = odrive.find_any()
    axes = [odrv0.axis0, odrv0.axis1]

    print("Watching for MOTOR_ERROR_CONTROL_DEADLINE_MISSED (poll 1 ms)...")
    while True:
        t0 = time.perf_counter()
        for i, ax in enumerate(axes):
            err = ax.motor.error
            if err & DEADLINE:
                dt_ms = (time.perf_counter() - t0) * 1000
                print(f"\n*** Deadline missed on axis{i} motor, poll slice ~{dt_ms:.3f} ms ***")
                print(f"axis{i} motor.error = 0x{err:08x}")
                for j, ax2 in enumerate(axes):
                    dump_timing_log(ax2.motor, f"axis{j} motor.timing_log")
                return
        time.sleep(POLL_S)

if __name__ == "__main__":
    main()