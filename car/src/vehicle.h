/* Vehicle state read over OBD-II (README step 3). */
#ifndef VEHICLE_H
#define VEHICLE_H

/* Step 3 precondition: true when the car is parked, so an exit may start. */
bool vehicle_is_parked(void);

#endif /* VEHICLE_H */
