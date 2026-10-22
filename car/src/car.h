/* Events shared by the car's Bluetooth callbacks and its main thread. */
#ifndef CAR_H
#define CAR_H

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define EV_CONNECTED    BIT(0)
#define EV_CONN_FAILED  BIT(1)
#define EV_DISCONNECTED BIT(2)
#define EV_SECURED      BIT(3)
#define EV_SEC_FAILED   BIT(4)
#define EV_MTU_DONE     BIT(5)
#define EV_DISCOVERED   BIT(6)
#define EV_SUBSCRIBED   BIT(7)
#define EV_WRITE_DONE   BIT(8)
#define EV_RX           BIT(9)

extern struct k_event car_events;

/*
 * Waits until any of `bits`, or a disconnect, is posted. Returns the bits seen,
 * or 0 on timeout. Clear a bit before starting the operation that posts it.
 */
static inline uint32_t car_wait(uint32_t bits, k_timeout_t timeout)
{
	return k_event_wait(&car_events, bits | EV_DISCONNECTED, false, timeout);
}

#endif /* CAR_H */
