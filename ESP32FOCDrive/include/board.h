#pragma once

/**
 * Board-level pins shared by both axes. M_EN (GPIO12) feeds BOTH gate
 * drivers, so it cannot be a per-driver enable pin: disabling one axis would
 * cut the other. Drivers are built without an enable pin and this refcount
 * owns M_EN instead.
 */
void boardInit();
/** delta = +1 when an axis arms, -1 when it disarms. Clamped at zero. */
void boardMotorPowerRef(int delta);
bool boardMotorPowered();
/** Hold GPIO0 low across a restart so the ROM enters download mode. */
void boardEnterDownload();
