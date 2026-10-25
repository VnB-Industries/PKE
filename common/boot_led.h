/* Short animation on the XIAO's onboard RGB LED, played at every boot. */
#ifndef BOOT_LED_H
#define BOOT_LED_H

/*
 * Starts the animation on the system workqueue and returns at once, so it
 * never delays Bluetooth start-up. Both boards reboot when they wake from
 * System OFF, so it also marks each wake. The LED ends off.
 */
void boot_led_play(void);

#endif /* BOOT_LED_H */
