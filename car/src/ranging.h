/* UWB ranging with the fob over the DW3000 (README steps 4 and 7). */
#ifndef RANGING_H
#define RANGING_H

#include <stdint.h>

#include "pke_protocol.h"

/*
 * Runs one DS-TWR exchange whose STS is seeded from key and stores the
 * distance. Returns 0, or -ENOTSUP while no DW3000 is fitted.
 */
int ranging_measure(const uint8_t key[PKE_KEY_LEN], uint32_t *distance_cm);

#endif /* RANGING_H */
