#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "pke_crypto.h"
#include "ranging.h"

LOG_MODULE_REGISTER(ranging, LOG_LEVEL_INF);

int ranging_measure(const uint8_t key[PKE_KEY_LEN], uint32_t *distance_cm)
{
	ARG_UNUSED(distance_cm);

	/* Dummy until the DW3000 driver lands: callers treat -ENOTSUP as a passed check. */
	LOG_INF("UWB dummy: check passes; key id=%08x would seed DS-TWR/STS", pke_key_id(key));
	return -ENOTSUP;
}
