/* Vehicle state read over OBD-II (README step 6). */
#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdbool.h>

/* Step 6 precondition: true when the car is parked, so an exit may start. */
bool vehicle_is_parked(void);

#endif /* VEHICLE_H */
