/* Motion sleep: the ADXL362 keeps the fob awake while it moves, System OFF otherwise. */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

/* No ADXL362 interrupt for this long puts the fob in System OFF. */
#define MOTION_IDLE_TIMEOUT_S 60

/*
 * Configures the ADXL362 as an autonomous motion switch and starts the idle
 * timer. on_idle runs from the system workqueue once no interrupt has come for
 * MOTION_IDLE_TIMEOUT_S. Returns 0, or a negative errno if the chip doesn't
 * answer: the fob must then never power off, since nothing could wake it.
 */
int motion_init(void (*on_idle)(void));

/* True while the ADXL362 reports the fob as moving (INT1 high). */
bool motion_active(void);

/* Milliseconds since the ADXL362 last changed state (started or stopped moving). */
int64_t motion_rest_ms(void);

/* Restarts the idle timer, for when the caller isn't ready to sleep yet. */
void motion_defer(void);

/* Enters System OFF; INT1 going high wakes the fob, which then reboots. Doesn't return. */
void motion_poweroff(void);

#endif /* MOTION_H */
