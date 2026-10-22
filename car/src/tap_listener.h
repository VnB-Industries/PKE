/* Window tap gestures, heard only while the fob is detected. */
#ifndef TAP_LISTENER_H
#define TAP_LISTENER_H

/* Installs the I2S driver, stopped, and the listener tasks. Returns 0 or a negative errno; the listener stays off on error. */
int tap_listener_init(void);

/* Powers the mic and starts listening: double tap = unlock, triple tap = lock. */
void tap_listener_start(void);

/* Stops the I2S clock, which powers the mic down, and drops any tap in progress. */
void tap_listener_stop(void);

#endif /* TAP_LISTENER_H */
