/* Window tap gestures, heard only while an authenticated fob is connected. */
#ifndef TAP_LISTENER_H
#define TAP_LISTENER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Checks the I2S device. Returns 0 or a negative errno; the listener stays off on error. */
int tap_listener_init(void);

/* Powers the mic and starts listening: double tap = unlock, triple tap = lock. */
void tap_listener_start(void);

/* Stops the I2S clock, which powers the mic down, and drops any tap in progress. */
void tap_listener_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TAP_LISTENER_H */
