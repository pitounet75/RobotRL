#pragma once

/**
 * Board-level pins shared by both axes. M_EN (GPIO12) feeds BOTH gate
 * drivers, so it cannot be a per-driver enable pin: disabling one axis would
 * cut the other. Drivers are built without an enable pin and this refcount
 * owns M_EN instead.
 */
void boardInit();
/** delta = +1 when an axis arms, -1 when it disarms. Clamped at zero. Not
 * safe to call bare: the FOC task's failsafe disarm can preempt the CLI in
 * the middle of an update (same core 1, higher priority), and the design must
 * stay correct if the task ever moves core again -- see boardMotorPowerLock(). */
void boardMotorPowerRef(int delta);
bool boardMotorPowered();
/**
 * Guards the M_EN refcount together with the caller's own armed flag. Before
 * the task-7 command failsafe, axisArm()/axisDisarm() (axis.cpp) were only
 * ever reachable from core 1 (the CLI): the refcount was single-threaded by
 * construction. The failsafe now calls axisFailsafeDisarm() (axis.cpp, which
 * shares axisDisarm()'s guarded body) from the FOC task too -- both the CLI
 * and the FOC task run on core 1 today, but the FOC task preempts the CLI at
 * any instruction, so "read power_refs, add delta, write it back" in
 * boardMotorPowerRef() can interleave between the two and lose an update --
 * M_EN then stays high with both axes disarmed, or drops while one is still
 * being driven. axisArm()/axisDisarm() take this lock around their whole
 * test-and-set of ax.armed plus the boardMotorPowerRef() call, so "decide to
 * (dis)arm" and "update the refcount" happen as one indivisible step
 * regardless of whether the caller and the task end up on the same core or
 * different ones. */
void boardMotorPowerLock();
void boardMotorPowerUnlock();
/** Hold GPIO0 low across a restart so the ROM enters download mode. */
void boardEnterDownload();
