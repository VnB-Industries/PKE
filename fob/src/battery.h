/* Onboard LiPo: charger status and cell voltage, read once for the boot log. */
#ifndef BATTERY_H
#define BATTERY_H

/*
 * Logs whether the BQ25101 is charging and the cell voltage. The voltage
 * divider is switched on only for the reading, so it draws nothing in sleep.
 */
void battery_log_state(void);

#endif /* BATTERY_H */
